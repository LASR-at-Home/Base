"""Numerical IK for the arm, built from the robot's own URDF (/robot_description).

Plain python + numpy (no ROS), so it can be tested offline. The chain, joint axes,
offsets and limits all come from the URDF; nothing about the arm is hardcoded.
"""
import itertools
import math
import xml.etree.ElementTree as ET

import numpy as np


def _floats(s, default):
    return np.array([float(v) for v in s.split()]) if s else np.array(default, dtype=float)


def _rpy(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _rot(axis, a):
    """Rotation of angle a about unit vector axis (Rodrigues)."""
    x, y, z = axis
    c, s, v = math.cos(a), math.sin(a), 1.0 - math.cos(a)
    return np.array([
        [c + x * x * v, x * y * v - z * s, x * z * v + y * s],
        [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
        [z * x * v - y * s, z * y * v + x * s, c + z * z * v],
    ])


def _tf(xyz, R):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T


def rot_error(R_cur, R_des):
    """Rotation vector (base frame) that turns R_cur towards R_des."""
    return 0.5 * sum(np.cross(R_cur[:, i], R_des[:, i]) for i in range(3))


def point_forward(R):
    """R turned the least amount needed so its x axis (the gripper's approach
    axis) points straight ahead along base +x, level with the floor."""
    a = R[:, 0]
    k = np.cross(a, [1.0, 0.0, 0.0])
    s, c = np.linalg.norm(k), a[0]
    if s < 1e-9:
        return R.copy()
    return _rot(k / s, math.atan2(s, c)) @ R


class ArmIK:
    def __init__(self, urdf_xml, joint_names, base="base_footprint", tip="gripper_grasping_frame"):
        root = ET.fromstring(urdf_xml)
        by_child = {}
        for j in root.findall("joint"):
            origin = j.find("origin")
            axis = j.find("axis")
            limit = j.find("limit")
            soft = j.find("safety_controller")
            xyz = _floats(origin.get("xyz") if origin is not None else None, [0, 0, 0])
            rpy = _floats(origin.get("rpy") if origin is not None else None, [0, 0, 0])
            ax = _floats(axis.get("xyz") if axis is not None else None, [1, 0, 0])
            lower = float(limit.get("lower", "-inf")) if limit is not None else -math.inf
            upper = float(limit.get("upper", "inf")) if limit is not None else math.inf
            # the controllers enforce the soft limits where the URDF gives them
            if soft is not None:
                lower = max(lower, float(soft.get("soft_lower_limit", lower)))
                upper = min(upper, float(soft.get("soft_upper_limit", upper)))
            by_child[j.find("child").get("link")] = {
                "name": j.get("name"),
                "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "T": _tf(xyz, _rpy(*rpy)),
                "axis": ax / np.linalg.norm(ax) if np.linalg.norm(ax) > 0 else np.array([1.0, 0.0, 0.0]),
                "lower": lower,
                "upper": upper,
            }

        chain, link = [], tip
        while link != base:
            if link not in by_child:
                raise ValueError(f"no joint chain from {base} to {tip} (stuck at {link})")
            chain.append(by_child[link])
            link = by_child[link]["parent"]
        chain.reverse()

        self.names = list(joint_names)
        n = len(self.names)
        self.lower = np.full(n, -math.inf)
        self.upper = np.full(n, math.inf)
        # Fixed joints are folded into the next moving joint's offset.
        self._segs, pre = [], np.eye(4)
        for j in chain:
            pre = pre @ j["T"]
            if j["type"] == "fixed":
                continue
            if j["name"] not in self.names:
                raise ValueError(f"moving joint {j['name']} on the chain is not in joint_names")
            i = self.names.index(j["name"])
            if j["type"] != "continuous":
                self.lower[i], self.upper[i] = j["lower"], j["upper"]
            self._segs.append((pre, i, j["type"] == "prismatic", j["axis"]))
            pre = np.eye(4)
        self._tip = pre
        missing = set(range(n)) - {s[1] for s in self._segs}
        if missing:
            raise ValueError(f"joints not on the chain: {[self.names[i] for i in missing]}")

        # Finger pads: the collision boxes of the "finger" links hanging off the tip's
        # parent, as the (min, max) x they span in the tip frame (x = approach), or None;
        # pad_pts are their corners in the tip frame (fingers at zero opening).
        self.pads, self.pad_pts = None, None
        links = {l.get("name"): l for l in root.findall("link")}
        tip_joint = by_child[tip]
        to_tip = np.linalg.inv(tip_joint["T"])
        pts = []
        for child, j in by_child.items():
            if j["parent"] != tip_joint["parent"] or "finger" not in child or child not in links:
                continue
            for col in links[child].findall("collision"):
                box, o = col.find("geometry/box"), col.find("origin")
                if box is None:
                    continue
                Tc = _tf(_floats(o.get("xyz") if o is not None else None, [0, 0, 0]),
                         _rpy(*_floats(o.get("rpy") if o is not None else None, [0, 0, 0])))
                half = _floats(box.get("size"), [0, 0, 0]) / 2.0
                for c in itertools.product(*[(-h, h) for h in half]):
                    pts.append((to_tip @ j["T"] @ Tc @ np.array([*c, 1.0]))[:3])
        if pts:
            self.pad_pts = np.array(pts)
            self.pads = (float(self.pad_pts[:, 0].min()), float(self.pad_pts[:, 0].max()))

    def clip(self, q, margin=0.01):
        return np.minimum(np.maximum(q, self.lower + margin), self.upper - margin)

    def _fk_jac(self, q):
        """Tip pose, tip Jacobian, and (joint index, prismatic, axis, origin) of every
        moving joint along the chain, all in the base frame."""
        T = np.eye(4)
        frames = []
        for pre, i, prismatic, axis in self._segs:
            T = T @ pre
            z, p = T[:3, :3] @ axis, T[:3, 3].copy()
            frames.append((i, prismatic, z, p))
            M = np.eye(4)
            if prismatic:
                M[:3, 3] = axis * q[i]
            else:
                M[:3, :3] = _rot(axis, q[i])
            T = T @ M
        T = T @ self._tip
        J = np.zeros((6, len(self.names)))
        for i, prismatic, z, p in frames:
            if prismatic:
                J[:3, i] = z
            else:
                J[:3, i] = np.cross(z, T[:3, 3] - p)
                J[3:, i] = z
        return T, J, frames

    def fk(self, q):
        return self._fk_jac(np.asarray(q, dtype=float))[0]

    def joint_origins(self, q):
        """{joint name: its origin in the base frame} for the moving joints."""
        frames = self._fk_jac(np.asarray(q, dtype=float))[2]
        return {self.names[i]: p for i, _, _, p in frames}

    def solve(self, q0, pos, R=None, keep_up=None, tol_pos=0.003, tol_rot=0.03,
              rot_weight=0.5, iters=300, restarts=4):
        """Joint vector putting the tip at pos (and orientation R, if given), started
        from q0 so the answer stays close to the current arm. keep_up maps joint names
        to (min height, x beyond which it applies) for that joint's origin, e.g. elbow
        and wrist above a table that starts at some x. None if unreachable."""
        pos = np.asarray(pos, dtype=float)
        keep_up = keep_up or {}
        up = [(k,) + tuple(keep_up[self.names[i]]) for k, (_, i, _, _) in enumerate(self._segs)
              if self.names[i] in keep_up]
        rng = np.random.default_rng(0)
        q0 = self.clip(np.asarray(q0, dtype=float))
        for attempt in range(restarts + 1):
            q = q0 if attempt == 0 else self.clip(q0 + rng.normal(0.0, 0.3, q0.shape))
            for _ in range(iters):
                T, J, frames = self._fk_jac(q)
                ep = pos - T[:3, 3]
                er = np.zeros(3) if R is None else rot_error(T[:3, :3], R)
                rows, errs = [J[:3]], [ep]
                if R is not None:
                    rows.append(rot_weight * J[3:])
                    errs.append(rot_weight * er)
                low = 0.0
                for k, z_min, x_from in up:
                    pk = frames[k][3]
                    if pk[0] < x_from:
                        continue
                    low = max(low, z_min - pk[2])
                    if pk[2] < z_min + 0.01:   # active: push this point up
                        row = np.zeros(len(self.names))
                        for i, prismatic, z, p in frames[:k]:
                            row[i] = z[2] if prismatic else np.cross(z, pk - p)[2]
                        rows.append(row[None, :])
                        errs.append(np.array([z_min + 0.01 - pk[2]]))
                if np.linalg.norm(ep) < tol_pos and np.linalg.norm(er) < tol_rot and low < 0.003:
                    return q
                Jx, e = np.vstack(rows), np.concatenate(errs)
                dq = Jx.T @ np.linalg.solve(Jx @ Jx.T + 0.05 ** 2 * np.eye(len(e)), e)
                step = np.max(np.abs(dq))
                if step > 0.2:
                    dq *= 0.2 / step
                q = self.clip(q + dq)
        return None

    def solutions(self, q_ref, pos, R=None, keep_up=None, seeds=120, enough=8, give_up=None):
        """Several answers for the same target, from a local search at q_ref plus many
        spread-out starting postures (e.g. elbow-up as well as elbow-down), sorted by
        how much joint motion each needs from q_ref. With give_up, stop after that many
        starting postures if none has found one (the target is out of reach)."""
        q_ref = np.asarray(q_ref, dtype=float)
        found = []
        q = self.solve(q_ref, pos, R, keep_up)
        if q is not None:
            found.append(q)
        rng = np.random.default_rng(1)
        lo, hi = self.clip(np.full(len(self.names), -1e9)), self.clip(np.full(len(self.names), 1e9))
        for i in range(seeds):
            if len(found) >= enough or (give_up is not None and not found and i >= give_up):
                break
            seed = lo + rng.random(len(self.names)) * (hi - lo)
            q = self.solve(seed, pos, R, keep_up, iters=200, restarts=0)
            if q is not None:
                found.append(q)
        return sorted(found, key=lambda q: np.abs(q - q_ref).max())

    def solve_global(self, q_ref, pos, R=None, keep_up=None):
        """The answer needing the least joint motion from q_ref, or None."""
        found = self.solutions(q_ref, pos, R, keep_up)
        return found[0] if found else None
