"""Put the held object down on a free spot of a shelf: the grasp run backwards.

From the shelf's observe pose (config pick_and_place.place.observe_pose, ~1.1 m in front):
square up to the shelf, find each board's top and front edge and what already stands on
it with the depth camera, pick the free spot nearest the robot's centre line (an empty
board first), then plan, drive in on the board's front edge and reach in straight, as the
grasp does, with the board as the table and the board above as a ceiling. Open the
gripper, back out, tuck.
"""
import math
import time

import numpy as np
import yasmin
import yasmin_ros
from geometry_msgs.msg import Point, Pose, Quaternion

from pick_and_place.states.arm_ik import point_forward
from pick_and_place.states.grasp_object import GraspObject, GRIPPER_OPEN, INIT_JOINTS_RIGHT


def _param(node, name, default):
    try:
        v = node.get_parameter(name).value
        return default if v is None else v
    except Exception:
        return default


class ShelfLocation(yasmin.State):
    """After a grasp: send the robot to the shelf's observe pose if it holds something
    and the place step is on ("shelf"), else on to the old destination ("other")."""

    def __init__(self):
        super().__init__(outcomes=["shelf", "other"])
        self.node = yasmin_ros.logger_node

    def execute(self, blackboard):
        held = blackboard["held_object"] if "held_object" in blackboard else None
        if not held or not bool(_param(self.node, "pick_and_place.place.enable", True)):
            return "other"
        dest = blackboard["destination"] if "destination" in blackboard else ""
        if dest != "cabinet" and not bool(_param(self.node, "pick_and_place.place.all_objects", True)):
            return "other"
        b = "pick_and_place.place.observe_pose."
        g = lambda k: float(_param(self.node, b + k, 0.0))
        blackboard["location"] = Pose(
            position=Point(x=g("position.x"), y=g("position.y"), z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=g("orientation.z"), w=g("orientation.w")),
        )
        yasmin.YASMIN_LOG_INFO(f"Holding the object: going to the shelf at ({g('position.x'):.2f}, {g('position.y'):.2f})")
        return "shelf"


class PlaceObject(GraspObject):
    def __init__(self):
        super().__init__()
        self._ceil, self._yc, self._half_w, self._shelf_depth = 9.0, 0.0, 9.0, 9.0
        self._hold, self._holding_now = None, True

    # ── the shelf as the grasp's "table", plus a ceiling, side panels and a back ──────
    def _arm_points(self, ik, q, holding):
        pts = super()._arm_points(ik, q, False)
        if holding and self._hold:
            T = ik.fk(q)
            c = T[:3, 3] + self._pad_mid * T[:3, 0]   # the held object's axis, mid-pad
            w = self._hold["width"] / 2.0
            for dz in (-self._hold["below"], self._hold["above"]):
                for dy in (-w, 0.0, w):
                    pts.append((c + np.array([0.0, dy, dz]), 0.01, True))
        return pts

    def _hits(self, p, clearance, body, torso, edge, table_z):
        if super()._hits(p, clearance, body, torso, edge, table_z):
            return True
        if p[0] > edge - 0.01:   # inside the shelf
            return (p[2] > self._ceil - max(clearance, 0.015)
                    or abs(p[1] - self._yc) > self._half_w - max(clearance, 0.01)
                    or p[0] > edge + self._shelf_depth - 0.03)
        return False

    def _path_ok(self, ik, qs, edge, table_z, holding=True, n=25):
        # every move is checked with the object in the hand until it has been let go
        return super()._path_ok(ik, qs, edge, table_z, self._holding_now, n)

    def _edge_line(self, table_z, r):
        """The board's front edge as x = a + b*y from its 2 cm front face (the table's
        face band would be too deep for a board)."""
        pts, _, _ = self._depth_points()
        if pts is None:
            return None
        sel = pts[(np.abs(pts[:, 1] - self._yc) < min(self._half_w, 0.6) - 0.03) & (pts[:, 0] > r + 0.05)
                  & (pts[:, 2] > table_z - 0.022) & (pts[:, 2] < table_z - 0.003)]
        if len(sel) < 30 or np.ptp(sel[:, 1]) < 0.15:
            return None
        ys, xs = sel[:, 1], sel[:, 0]
        near = xs < np.percentile(xs, 5) + 0.05   # the face, not the shelf's back
        ys, xs = ys[near], xs[near]
        if len(ys) < 20:
            return None
        a, b = 0.0, 0.0
        inl = np.ones(len(ys), bool)
        for _ in range(3):
            a, b = np.linalg.lstsq(np.stack([np.ones(inl.sum()), ys[inl]], 1), xs[inl], rcond=None)[0]
            inl = np.abs(xs - (a + b * ys)) < 0.01
            if inl.sum() < 20 or np.ptp(ys[inl]) < 0.15:
                return None
        return (float(a), float(b)) if abs(b) < 1.5 else None

    # ── seeing the shelf ──────────────────────────────────────────────────────
    def _scan(self, boards, gap, width, r):
        """Look at each board: [(z, front x, centre y, half width, spot y or None,
        occupied point count)], the depth points seen, all in base_footprint."""
        out, seen = [], []
        for zc in boards:
            self._look_at(r + 0.95, 0.0, zc + 0.05)
            pts = self._depth_points(step=2)[0]
            if pts is None:
                continue
            seen.append(pts)
            band = pts[(np.abs(pts[:, 2] - zc) < 0.04) & (pts[:, 0] > r + 0.1) & (np.abs(pts[:, 1]) < 0.8)]
            if len(band) < 50:
                yasmin.YASMIN_LOG_WARN(f"Shelf: can't see a board near z={zc:.2f}")
                continue
            h, e = np.histogram(band[:, 2], bins=np.arange(zc - 0.04, zc + 0.045, 0.01))
            k = int(np.argmax(h))
            top = band[(band[:, 2] >= e[k] - 0.005) & (band[:, 2] < e[k + 1] + 0.005)]
            z = float(np.median(top[:, 2]))
            top = top[np.abs(top[:, 2] - z) < 0.007]
            front = float(np.percentile(top[:, 0], 2))
            lo, hi = np.percentile(top[:, 1], 2), np.percentile(top[:, 1], 98)
            yc, half = ((lo + hi) / 2.0, (hi - lo) / 2.0) if 0.25 < (hi - lo) / 2.0 < 0.6 else (0.0, width / 2.0)
            occ = pts[(pts[:, 2] > z + 0.015) & (pts[:, 2] < z + min(0.30, gap - 0.03))
                      & (pts[:, 0] > front + 0.02) & (pts[:, 0] < front + 0.30)
                      & (np.abs(pts[:, 1] - yc) < half - 0.02)]
            need = self._hold["width"] / 2.0 + 0.04
            ys = np.arange(yc - half + need, yc + half - need + 1e-6, 0.01)
            free = [y for y in ys if not np.any(np.abs(occ[:, 1] - y) < need)]
            spot = min(free, key=abs) if free else None
            yasmin.YASMIN_LOG_INFO(
                f"Shelf board z={z:.2f}: front x={front:.2f}, centre y={yc:+.2f}, {2 * half:.2f} m wide, "
                f"{len(occ)} points standing on it, free spot "
                + ("none" if spot is None else f"y={spot:+.2f}")
            )
            out.append((z, front, yc, half, spot, len(occ)))
        return out, seen

    def _square_up(self, z, front, r):
        """Turn so the board's front edge is square to the base (the drive goes straight in)."""
        for _ in range(2):
            line = self._look_edge(front, z)
            if line is None or abs(math.atan(line[1])) < 0.03:
                return line
            yasmin.YASMIN_LOG_INFO(f"Shelf edge is {math.degrees(math.atan(line[1])):+.1f} deg off square: turning")
            self._turn_by(-math.atan(line[1]))
        return self._look_edge(front, z)

    def _drive_in(self, d, line0, edge, table_z, r):
        """The grasp's drive: stop short, measure on the edge, drive the rest by it.
        (driven, expected move, tolerance)."""
        driven = self._drive_to(d - 0.15 if d > 0.30 else d)
        expect, tol = driven, 0.05 + 0.4 * driven
        if driven < d - 0.05:
            seen = self._look_edge(edge - driven, table_z)
            slip, moved = 1.0, 0.0
            if seen is not None:
                moved = self._edge_x(line0) - self._edge_x(seen)
                rest = min(d - moved, self._edge_x(seen) - r - 0.05)
                if moved > 0.10:
                    slip = min(1.4, max(0.7, driven / moved))
                yasmin.YASMIN_LOG_INFO(
                    f"Shelf edge says the base moved {moved:.3f} m so far (odometry {driven:.3f} m): "
                    f"driving the last {max(rest, 0.0):.3f} m (odometry x{slip:.2f})"
                )
            else:
                rest = d - driven
            last = self._drive_to(rest * slip) if rest > 0.01 else 0.0
            if seen is not None:
                expect, tol = moved + last / slip, 0.05 + 0.2 * last / slip
            else:
                expect, tol = driven + last, 0.05 + 0.4 * (driven + last)
            driven += last
        return driven, expect, tol

    def _fail(self, ik, path, driven, edge, z, why):
        yasmin.YASMIN_LOG_WARN(f"{why}: not placing")
        if path is not None:
            self._retreat(ik, path, driven, edge, z, True)
        return "failed"

    def execute(self, blackboard) -> str:
        held = blackboard["held_object"] if "held_object" in blackboard else None
        if not held:
            yasmin.YASMIN_LOG_WARN("Not holding anything: nothing to place")
            return "failed"
        self._setup()
        self._setup_ik()
        t0 = time.time()
        while not self._joints and time.time() - t0 < 5.0:
            time.sleep(0.1)
        if not self._wait_for_tf():
            return "failed"
        ik = self._get_ik()
        if ik is None or not self._ik_matches_tf(ik):
            yasmin.YASMIN_LOG_WARN("No usable arm model: not placing")
            return "failed"

        P = "pick_and_place.place."
        boards = [float(v) for v in self._param(P + "board_z", [0.44, 0.81])]
        gap = float(self._param(P + "gap", 0.35))
        width = float(self._param(P + "inner_width", 0.86))
        self._shelf_depth = float(self._param(P + "depth", 0.40))
        drop = float(self._param(P + "drop", 0.015))
        behind = float(self._param(P + "behind_edge", 0.13))
        self._r = r = float(self._param("pick_and_place.grasp.base_radius", 0.28))
        tol = float(self._param("pick_and_place.grasp.tolerance", 0.03))
        self._hold, self._holding_now = held, True
        self._obstacles, self._edge0, self._line0 = np.zeros((0, 3)), 0.0, None
        lo, hi = ik.pads if ik.pads else (0.0, 0.10)
        self._pad_mid = (lo + hi) / 2.0
        R0 = point_forward(ik.fk(INIT_JOINTS_RIGHT)[:3, :3])
        yasmin.YASMIN_LOG_INFO(
            f"Place 1/6: look at the shelf (holding a {100 * held['width']:.1f} cm object, "
            f"{100 * held['below']:.1f} cm below / {100 * held['above']:.1f} cm above the grip)"
        )

        # 1. Square up to the shelf (on the board nearest the gripper's carrying height),
        #    then see every board, its front and what stands on it, from there.
        found, _ = self._scan(boards, gap, width, r)
        if not found:
            yasmin.YASMIN_LOG_WARN("Can't see the shelf's boards: not placing")
            return "failed"
        z_sq, f_sq, self._yc, self._half_w = max(found, key=lambda s: s[0])[:4]
        if self._square_up(z_sq, f_sq, r) is None:
            yasmin.YASMIN_LOG_WARN("Can't see the shelf's front edge: not placing")
            return "failed"
        found, seen = self._scan(boards, gap, width, r)
        # an empty board first, then the most room; reachable heights only
        options = sorted((s for s in found if s[4] is not None), key=lambda s: (s[5] > 30, s[5], -s[0]))
        if not options:
            yasmin.YASMIN_LOG_WARN("No free spot on any board: not placing")
            return "failed"

        for z, front, self._yc, self._half_w, spot_y, n_occ in options:
            ceil = min([b - 0.02 for b, *_ in found if b > z + 0.05] + [z + gap])
            self._ceil = ceil
            centre = np.array([front + behind, spot_y, z + drop + held["below"]])
            g = centre - np.array([self._pad_mid, 0.0, 0.0])
            standoff = behind + 0.02 + held["width"] / 2.0
            carry_z = g[2] + 0.03
            if carry_z + held["above"] > ceil - 0.015:
                yasmin.YASMIN_LOG_INFO(f"Board z={z:.2f}: the object doesn't fit under the board above ({ceil:.2f})")
                continue
            obs = np.vstack(seen) if seen else np.zeros((0, 3))
            obs = obs[(obs[:, 2] > z + 0.015) & (obs[:, 0] > front - 0.02)
                      & (np.hypot(obs[:, 0] - centre[0], obs[:, 1] - centre[1]) > held["width"] / 2.0 + 0.04)]
            self._obstacles = (np.unique(np.floor(obs / 0.02), axis=0) + 0.5) * 0.02 if len(obs) else obs
            self._edge0 = edge = front
            max_drive = max(0.0, min(1.0, edge - r - 0.05))
            yasmin.YASMIN_LOG_INFO(
                f"Place 2/6: planning for board z={z:.2f} ({'empty' if n_occ <= 30 else f'{n_occ} points on it'}), "
                f"spot x={centre[0]:.2f} y={centre[1]:+.2f}, ceiling {ceil:.2f}"
            )
            plan = self._plan(ik, np.array(self._arm_joints()), g, R0, edge, z, max_drive, standoff, carry_z, False)
            if plan is None:
                yasmin.YASMIN_LOG_INFO(f"Board z={z:.2f}: no safe arm path")
                continue
            break
        else:
            yasmin.YASMIN_LOG_WARN("No board the arm can place on safely: not placing")
            return "failed"
        d, path = plan
        line0 = self._line0 = self._look_edge(edge, z)
        if line0 is None:
            return self._fail(ik, None, 0.0, edge, z, "Can't see the board's front edge to measure the drive")
        yasmin.YASMIN_LOG_INFO(f"Place 3/6: plan — drive {d:.2f} m, arm via {len(path) - 2} waypoint(s), then reach in")

        # 2. Arm out along the checked path, then drive in, measured on the board's edge.
        for i, q in enumerate(path[1:], start=1):
            if not self._move(list(q), timeout=20.0, secs=5):
                for q_back in reversed(path[:i]):
                    self._move(list(q_back), timeout=20.0, secs=5)
                yasmin.YASMIN_LOG_WARN("Arm did not reach the planned pose: not placing")
                return "failed"
        yasmin.YASMIN_LOG_INFO("Place 4/6: drive in")
        driven, expect, dtol = self._drive_in(d, line0, edge, z, r)
        line1 = self._look_edge(edge - driven, z)
        if line1 is None:
            return self._fail(ik, path, driven, edge, z, "Couldn't see the board's edge after the drive")
        t, phi = self._moved(line0, line1)
        yasmin.YASMIN_LOG_INFO(
            f"Board edge: the base moved {t:.3f} m and turned {math.degrees(phi):+.1f} deg "
            f"(odometry {driven:.3f} m; expected {expect:.3f} m)"
        )
        if abs(t - expect) > dtol or abs(phi) > 0.15:
            return self._fail(ik, path, driven, edge, z, f"That is {t - expect:+.3f} m from what was expected")
        rot = lambda v: np.array([math.cos(phi) * v[0] + math.sin(phi) * v[1],
                                  -math.sin(phi) * v[0] + math.cos(phi) * v[1], v[2]])
        shift = np.array([t * math.cos(phi / 2.0), t * math.sin(phi / 2.0), 0.0])
        c_now = rot(centre - shift)
        self._yc = float(rot(np.array([front + self._shelf_depth / 2.0, self._yc, 0.0]) - shift)[1])
        driven = self._edge_x(line0) - self._edge_x(line1)
        g = c_now - np.array([self._pad_mid, 0.0, 0.0])
        start = np.array([g[0] - standoff, g[1], g[2]])

        # 3. Reach in straight, check, let go, back out.
        yasmin.YASMIN_LOG_INFO("Place 5/6: reach in and let go")
        qs = self._reach_in(ik, self._arm_joints(), g, R0, standoff, carry_z, False, edge - driven, z)
        if qs is None:
            return self._fail(ik, path, driven, edge, z, f"Spot out of reach after driving {driven:.2f} m")
        prev = self._arm_joints()
        for qw in qs:
            step = np.linalg.norm(ik.fk(qw)[:3, 3] - ik.fk(prev)[:3, 3])
            self._move(list(qw), timeout=10.0, secs=2 if step > 0.05 else 1)
            prev = qw
        ee = self._get_ee()
        err = None if ee is None else g - np.array([ee.x, ee.y, ee.z])
        keep = self._keep(z, edge - driven)
        if err is None or np.linalg.norm(err) > tol:
            q_out = ik.solve(self._arm_joints(), start, R0, keep)
            if q_out is not None:
                self._move(list(q_out), timeout=10.0, secs=2)
            return self._fail(ik, path, driven, edge, z, "Gripper did not get to the spot")
        yasmin.YASMIN_LOG_INFO(
            f"At the spot: gripper error dx={err[0]:+.3f} dy={err[1]:+.3f} dz={err[2]:+.3f}; opening"
        )
        self._gripper_cmd(GRIPPER_OPEN)
        time.sleep(1.0)
        self._holding_now = False
        blackboard["held_object"] = None
        q_out = ik.solve(self._arm_joints(), start, R0, keep)
        if q_out is not None:
            self._move(list(q_out), timeout=10.0, secs=2)
        yasmin.YASMIN_LOG_INFO("Place 6/6: retract and tuck arm")
        self._retreat(ik, path, driven, edge, z, False)
        yasmin.YASMIN_LOG_INFO(f"Placed on the board at z={z:.2f}.")
        return "succeeded"
