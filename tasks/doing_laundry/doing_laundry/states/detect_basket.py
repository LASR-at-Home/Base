"""
basket_grasp_state.py — Floor laundry-basket perception as a YASMIN state.

Detects a basket standing on the floor, decides empty vs. occupied, and for a
single T-shirt emits collision-free points only (no manipulation):
    pre_grasp (vertical approach above rim) -> grasp (cloth fold) -> lift.

Geometry-first (depth); colour guards against grabbing basket/rim plastic.
Frame: base_footprint (Z up, floor ~ z=0; basket always on the floor).

This file exports two things:
  BasketPerception  — plain helper; attaches subs/pubs/tf to an EXISTING node.
  DetectBasket      — YASMIN state; takes NO node (grabs the shared yasmin node),
                      so it slots in exactly like your other states: DetectBasket().

State outcomes : 'grasp_ready' | 'empty' | 'no_basket' | 'failed'
Blackboard out (on grasp_ready): basket_pose, basket_yaw,
    pre_grasp_pose, grasp_pose, lift_pose, grasp_yaw
"""

import math
import time
from dataclasses import dataclass

import numpy as np
import cv2
import rclpy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, Quaternion
from std_msgs.msg import String
from cv_bridge import CvBridge
import tf2_ros

try:
    from scipy.spatial import cKDTree
    def DBSCAN_labels(pts2d, eps, min_samples):
        n = len(pts2d)
        if n == 0:
            return np.array([], dtype=int)
        tree = cKDTree(pts2d)
        neigh = tree.query_ball_point(pts2d, eps)
        labels = np.full(n, -1, dtype=int)
        cid = 0
        for i in range(n):
            if labels[i] != -1 or len(neigh[i]) < min_samples:
                continue
            stack, labels[i] = [i], cid
            while stack:
                p = stack.pop()
                if len(neigh[p]) >= min_samples:
                    for q in neigh[p]:
                        if labels[q] == -1:
                            labels[q] = cid
                            stack.append(q)
            cid += 1
        return labels
    SKLEARN = True
except ImportError:
    SKLEARN = False

try:
    from yasmin import State
except ImportError:
    class State:                             # fallback shim if yasmin is absent
        def __init__(self, outcomes):
            self._outcomes = outcomes


DEPTH_MIN, DEPTH_MAX = 0.3, 3.0
SUBSAMPLE = 5

# Basket (known): height 31, opening 36.4 x 36.2 cm.
BASKET_H = 0.31
OPEN_X, OPEN_Y = 0.364, 0.362
SIZE_TOL = 0.10

WALL_BAND = 0.025        # shrink opening by this to drop wall/rim points
EMPTY_MIN_PTS = 60
EMPTY_MIN_CLOTH_H = 0.04
HANDLE_Z_OVER = 0.06     # above rim + this = handle arch, never cloth
HANDLE_CLEAR_R = 0.05    # column radius checked against handle

DBSCAN_EPS, DBSCAN_MIN = 0.04, 10
PEAK_LOCAL_R = 0.04
MAX_CANDIDATES = 40
COLOR_STD_MAX = 18.0     # Lab spread allowed inside one gripper-sized patch
WRINKLE_THRESH = 0.015   # local height that counts as a graspable fold
PINCH_BELOW = 0.01       # grab this far below the fold top
PRESS_DEPTH = 0.015      # flat cloth: push in this far before closing


@dataclass
class GraspResult:
    status: str
    reason: str = ''
    frame: str = 'base_footprint'
    basket_xy: tuple = (0.0, 0.0)
    basket_yaw: float = 0.0
    rim_z: float = 0.0
    grasp: tuple = None
    pre_grasp: tuple = None
    lift: tuple = None
    grasp_yaw: float = 0.0


def _to_local(x, y, cx, cy, yaw):
    dx, dy = x - cx, y - cy
    c, s = math.cos(yaw), math.sin(yaw)
    return dx * c + dy * s, -dx * s + dy * c


def _pca_yaw(xy):
    if xy.shape[0] < 3:
        return 0.0, 1.0
    c = xy - xy.mean(0)
    cov = (c.T @ c) / xy.shape[0]
    w, v = np.linalg.eigh(cov)
    major = v[:, int(np.argmax(w))]
    ratio = float(w.max() / (w.min() + 1e-9))
    return math.atan2(major[1], major[0]), ratio


def _quat_top_down(yaw):
    qx = (1.0, 0.0, 0.0, 0.0)
    half = yaw / 2.0
    qz = (0.0, 0.0, math.sin(half), math.cos(half))
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return Quaternion(
        x=w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        y=w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        z=w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w=w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _rot(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def _fit_opening(rim2d, c0, yaw0, tol=0.02):
    # Known-size template fit: lock the 36.4x36.2 rectangle onto whatever rim
    # edges are clean. Per-edge inlier counts let cloth-draped edges drop out;
    # 1 clean edge already closes the plane. Returns (cx, cy, yaw, conf, n_clean).
    if rim2d.shape[0] < 12:
        return float(c0[0]), float(c0[1]), float(yaw0), 0.0, 0
    hx, hy = OPEN_X / 2, OPEN_Y / 2
    c = np.array(c0, dtype=float)
    yaw = float(yaw0)
    cnt = {}
    exp = {}
    for _ in range(2):
        local = (rim2d - c) @ _rot(yaw)
        lx, ly = local[:, 0], local[:, 1]
        exp = {
            'xp': (np.abs(lx - hx) < tol) & (np.abs(ly) < hy + tol),
            'xn': (np.abs(lx + hx) < tol) & (np.abs(ly) < hy + tol),
            'yp': (np.abs(ly - hy) < tol) & (np.abs(lx) < hx + tol),
            'yn': (np.abs(ly + hy) < tol) & (np.abs(lx) < hx + tol),
        }
        cnt = {k: int(v.sum()) for k, v in exp.items()}
        dxs, dys = [], []
        if cnt['xp'] > 8: dxs.append(np.median(lx[exp['xp']]) - hx)
        if cnt['xn'] > 8: dxs.append(np.median(lx[exp['xn']]) + hx)
        if cnt['yp'] > 8: dys.append(np.median(ly[exp['yp']]) - hy)
        if cnt['yn'] > 8: dys.append(np.median(ly[exp['yn']]) + hy)
        dx = float(np.mean(dxs)) if dxs else 0.0
        dy = float(np.mean(dys)) if dys else 0.0
        c = c + _rot(yaw) @ np.array([dx, dy])
        be = max(cnt, key=cnt.get)
        if cnt[be] > 12:
            ed_yaw, _ = _pca_yaw(rim2d[exp[be]])
            yaw = ed_yaw - math.pi / 2 if be in ('xp', 'xn') else ed_yaw
            yaw = ((yaw + math.pi / 4) % (math.pi / 2)) - math.pi / 4
    clean = sum(1 for k in cnt if cnt[k] > 12)
    ratio = min(1.0, sum(cnt.values()) / rim2d.shape[0])
    conf = ratio * (1.0 if clean >= 2 else 0.5 if clean == 1 else 0.1)
    return float(c[0]), float(c[1]), float(yaw), float(conf), clean


def _split_handle(interior, rim_z):
    return interior[:0], interior   # HANDLE OFF
    # Handle (up-arch or folded-across) projects to a thin long line in XY;
    # cloth fills area. Detect the line by shape, then carve a corridor across
    # all z so descent/grasp avoid it regardless of handle height.
    if interior.shape[0] == 0:
        return interior[:0], interior
    elev = interior[interior[:, 2] > rim_z - 0.02]
    line = None
    if elev.shape[0] >= 8 and SKLEARN:
        lbl = DBSCAN_labels(elev[:, :2], 0.03, 8)
        for k in set(lbl) - {-1}:
            c = elev[lbl == k][:, :2]
            yaw_l, _ = _pca_yaw(c)
            d = np.array([math.cos(yaw_l), math.sin(yaw_l)])
            n = np.array([-d[1], d[0]])
            rel = c - c.mean(0)
            length = (rel @ d).max() - (rel @ d).min()
            width = (rel @ n).max() - (rel @ n).min()
            if length > 0.20 and width < 0.05:
                line = (c.mean(0), d)
                break
    if line is None:
        h = interior[interior[:, 2] > rim_z + HANDLE_Z_OVER]
        cl = interior[interior[:, 2] <= rim_z + HANDLE_Z_OVER]
        return h, cl
    p0, d = line
    rel = interior[:, :2] - p0
    perp = np.abs(rel[:, 0] * (-d[1]) + rel[:, 1] * d[0])
    m = perp < HANDLE_CLEAR_R
    return interior[m], interior[~m]


class BasketPerception:
    """Perception helper. Attaches subscriptions/publishers/tf to an existing
    (already-spinning) node. detect_once() runs the whole pipeline on demand."""

    def __init__(self, node, gripper_half=0.05, safe_margin=0.03,
                 approach_clear=0.15, lift_clear=0.22, search_z_max=0.70,
                 depth_topic='/head_front_camera/depth/image_raw',
                 rgb_topic='/head_front_camera/rgb/image_raw',
                 info_topic='/head_front_camera/rgb/camera_info'):
        self.node = node
        self.gripper_half = gripper_half
        self.safe_margin = safe_margin
        self.approach_clear = approach_clear
        self.lift_clear = lift_clear
        self.search_z_max = search_z_max

        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None
        self.depth = None
        self.rgb = None
        self.depth_frame = None
        self.depth_stamp = None
        self.depth_scale = 1.0
        self._enc = False
        self.base = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        node.create_subscription(Image, depth_topic, self._depth_cb, 1)
        node.create_subscription(CameraInfo, info_topic, self._cam_cb, 1)
        node.create_subscription(Image, rgb_topic, self._rgb_cb, 1)
        self.basket_pub = node.create_publisher(PoseStamped, '/basket_grasp/basket_pose', 10)
        self.grasp_pub = node.create_publisher(PoseStamped, '/basket_grasp/grasp_pose', 10)
        self.path_pub = node.create_publisher(PoseArray, '/basket_grasp/path', 10)
        self.state_pub = node.create_publisher(String, '/basket_grasp/state', 10)
        if not SKLEARN:
            node.get_logger().error('pip install scikit-learn')

    def _cam_cb(self, m):
        if self.fx is None:
            self.fx, self.fy, self.cx, self.cy = m.k[0], m.k[4], m.k[2], m.k[5]

    def _rgb_cb(self, m):
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(m, 'bgr8')
        except Exception:
            pass

    def _depth_cb(self, m):
        try:
            d = self.bridge.imgmsg_to_cv2(m, 'passthrough')
        except Exception:
            return
        if not self._enc:
            self.depth_scale = 0.001 if (m.encoding == '16UC1' or d.dtype == np.uint16) else 1.0
            self._enc = True
        self.depth = np.asarray(d, dtype=np.float32) * self.depth_scale
        self.depth_frame = m.header.frame_id
        self.depth_stamp = rclpy.time.Time.from_msg(m.header.stamp)

    def _base_frame(self, cam):
        if self.base:
            return self.base
        for cand in ('base_footprint', 'base_link', 'odom', 'map'):
            try:
                self.tf_buffer.lookup_transform(
                    cand, cam, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5))
                self.base = cand
                return cand
            except Exception:
                continue
        return None

    def _lookup(self, base, cam, stamp):
        # Use the depth frame's own stamp so points stay aligned while the head
        # moves; fall back to latest if that stamp isn't in the tf buffer yet.
        queries = ([stamp] if stamp is not None else []) + [rclpy.time.Time()]
        for q in queries:
            try:
                return self.tf_buffer.lookup_transform(
                    base, cam, q, timeout=rclpy.duration.Duration(seconds=0.3))
            except Exception:
                continue
        return None

    def _cloud(self):
        depth, cam = self.depth, self.depth_frame
        if depth is None or self.fx is None:
            return None, None
        base = self._base_frame(cam)
        if base is None:
            return None, None
        h, w = depth.shape[:2]
        vs, us = np.mgrid[0:h:SUBSAMPLE, 0:w:SUBSAMPLE]
        zs = depth[vs, us]
        m = (zs >= DEPTH_MIN) & (zs <= DEPTH_MAX) & np.isfinite(zs)
        us, vs, zs = us[m], vs[m], zs[m]
        if zs.size < 200:
            return None, None
        xc = (us - self.cx) * zs / self.fx
        yc = (vs - self.cy) * zs / self.fy
        tr = self._lookup(base, cam, self.depth_stamp)
        if tr is None:
            return None, None
        q, t = tr.transform.rotation, tr.transform.translation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ])
        pts = np.stack([xc, yc, zs], 1) @ R.T + np.array([t.x, t.y, t.z])
        if self.rgb is not None and self.rgb.shape[:2] == depth.shape[:2]:
            lab = cv2.cvtColor(self.rgb, cv2.COLOR_BGR2LAB)
            col = lab[vs, us].astype(np.float32)
        else:
            col = np.zeros((pts.shape[0], 3), np.float32)
        return np.hstack([pts, col]), base   # cols 0:3 xyz, 3:6 Lab

    def detect_once(self, timeout=2.0):
        if not SKLEARN:
            return GraspResult('failed', 'sklearn missing')
        t0 = time.time()
        while self.depth is None or self.fx is None:
            if time.time() - t0 > timeout:
                return GraspResult('failed', 'no depth/intrinsics')
            time.sleep(0.05)
        pts, base = self._cloud()
        if pts is None:
            return GraspResult('failed', 'tf/cloud')

        z = pts[:, 2]
        floor_z = float(np.clip(np.percentile(z, 3), -0.05, 0.10))

        # --- basket localization (cluster -> known-size opening fit) ---
        band = pts[(z > floor_z + 0.04) & (z < floor_z + self.search_z_max)]
        if band.shape[0] < 50:
            return GraspResult('no_basket', 'nothing standing on floor', frame=base, rim_z=floor_z + BASKET_H)
        labels = DBSCAN_labels(band[:, :2], DBSCAN_EPS, DBSCAN_MIN)
        best, best_rect, best_n = None, None, 0
        for lab in set(labels) - {-1}:
            cl = band[labels == lab]
            if cl.shape[0] < 40:
                continue
            if cl[:, 2].max() < floor_z + BASKET_H - 0.08:
                continue
            (rcx, rcy), (rw, rh), rang = cv2.minAreaRect(cl[:, :2].astype(np.float32))
            big = max(rw, rh)
            if big > OPEN_X + SIZE_TOL:
                continue
            if big < 0.6 * OPEN_X:
                continue
            if cl.shape[0] > best_n:
                best, best_rect, best_n = cl, ((rcx, rcy), (rw, rh), rang), cl.shape[0]
        if best is None:
            return GraspResult('no_basket', 'no basket-sized cluster', frame=base, rim_z=floor_z + BASKET_H)

        (rcx, rcy), _, rang = best_rect
        c0 = np.array([float(rcx), float(rcy)])
        yaw0 = ((math.radians(rang) + math.pi / 4) % (math.pi / 2)) - math.pi / 4

        rim_z = float(np.percentile(best[:, 2], 95))   # measured top opening (stack-safe)
        wall_h = rim_z - floor_z
        k = max(1, round(wall_h / BASKET_H))
        if abs(wall_h - k * BASKET_H) > 0.10:
            return GraspResult('no_basket', f'wall {wall_h:.2f}m not a multiple of {BASKET_H}',
                               frame=base, rim_z=rim_z)

        rim_band = best[(best[:, 2] > rim_z - 0.04) & (best[:, 2] < rim_z + 0.02)]
        cx, cy, yaw, conf, clean = _fit_opening(rim_band[:, :2], c0, yaw0, tol=0.04)
        if conf < 0.05:
            return GraspResult('failed', 'opening fit low-confidence (draped/occluded), re-view',
                               frame=base, basket_xy=(cx, cy), basket_yaw=yaw, rim_z=rim_z)

        # --- interior content (drop walls, split off handle) ---
        lx, ly = _to_local(pts[:, 0], pts[:, 1], cx, cy, yaw)
        hx, hy = OPEN_X / 2 - WALL_BAND, OPEN_Y / 2 - WALL_BAND
        inside = (np.abs(lx) <= hx) & (np.abs(ly) <= hy)
        inside &= (pts[:, 2] > floor_z + 0.02) & (pts[:, 2] < rim_z + 0.20)
        interior = pts[inside]
        if interior.shape[0] == 0:
            return GraspResult('empty', 'interior empty', frame=base,
                               basket_xy=(cx, cy), basket_yaw=yaw, rim_z=rim_z)
        handle, cloth = _split_handle(interior, rim_z)

        # --- empty vs occupied ---
        if cloth.shape[0] < EMPTY_MIN_PTS:
            return GraspResult('empty', 'too few cloth points', frame=base,
                               basket_xy=(cx, cy), basket_yaw=yaw, rim_z=rim_z)
        cloth_top = float(np.percentile(cloth[:, 2], 90))
        if cloth_top - floor_z < EMPTY_MIN_CLOTH_H:
            return GraspResult('empty', 'flat bottom only', frame=base,
                               basket_xy=(cx, cy), basket_yaw=yaw, rim_z=rim_z)

        # --- grasp peak inside collision-safe zone ---
        inset = self.gripper_half + self.safe_margin
        clx, cly = _to_local(cloth[:, 0], cloth[:, 1], cx, cy, yaw)
        safe_m = (np.abs(clx) <= OPEN_X / 2 - inset) & (np.abs(cly) <= OPEN_Y / 2 - inset)
        safe = cloth[safe_m]
        if safe.shape[0] < 5:
            return GraspResult('failed', 'cloth only near walls (no reachable grasp)',
                               frame=base, basket_xy=(cx, cy), basket_yaw=yaw, rim_z=rim_z)

        order = np.argsort(-safe[:, 2])[:MAX_CANDIDATES]
        r = self.gripper_half
        chosen = None
        for idx in order:
            p = safe[idx]
            d = np.linalg.norm(cloth[:, :2] - p[:2], axis=1)
            nb = cloth[d < r]
            if nb.shape[0] < 5:
                continue
            if float(np.linalg.norm(nb[:, 3:6].std(axis=0))) > COLOR_STD_MAX:
                continue
            gx, gy = float(nb[:, 0].mean()), float(nb[:, 1].mean())
            zmax = float(nb[:, 2].max())
            zmed = float(np.median(nb[:, 2]))
            if zmax - zmed > WRINKLE_THRESH:
                gz = zmax - PINCH_BELOW
            else:
                gz = zmed
            gz = max(gz, floor_z + 0.01)
            if handle.shape[0]:
                hd = np.linalg.norm(handle[:, :2] - np.array([gx, gy]), axis=1)
                if np.any((hd < HANDLE_CLEAR_R) &
                          (handle[:, 2] > gz) &
                          (handle[:, 2] < rim_z + self.lift_clear)):
                    continue
            ridge_yaw, _ = _pca_yaw(nb[:, :2])
            chosen = (gx, gy, gz, ridge_yaw + math.pi / 2)
            break
        if chosen is None:
            return GraspResult('failed', 'all grasp columns blocked by handle',
                               frame=base, basket_xy=(cx, cy), basket_yaw=yaw, rim_z=rim_z)

        gx, gy, gz, gyaw = chosen
        gz = gz + 0.12   # gripper_grasping_frame -> fingertip offset
        self.node.get_logger().info(f'[grasp] xy=({gx:.3f},{gy:.3f}) z={gz:.3f} floor={floor_z:.3f} rim={rim_z:.3f}')
        return GraspResult(
            'grasp_ready', 'ok', frame=base,
            basket_xy=(cx, cy), basket_yaw=yaw, rim_z=rim_z,
            grasp=(gx, gy, gz),
            pre_grasp=(gx, gy, rim_z + self.approach_clear),
            lift=(gx, gy, rim_z + self.lift_clear),
            grasp_yaw=gyaw,
        )

    def _pose(self, xyz, yaw):
        p = Pose()
        p.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
        p.orientation = _quat_top_down(yaw)
        return p

    def publish(self, r: GraspResult):
        self.state_pub.publish(String(data=f'{r.status}:{r.reason}'))
        now = self.node.get_clock().now().to_msg()
        if r.status in ('grasp_ready', 'empty'):
            bp = PoseStamped()
            bp.header.stamp = now
            bp.header.frame_id = r.frame
            bp.pose = self._pose((r.basket_xy[0], r.basket_xy[1], r.rim_z), r.basket_yaw)
            self.basket_pub.publish(bp)
        if r.status == 'grasp_ready':
            gp = PoseStamped()
            gp.header.stamp = now
            gp.header.frame_id = r.frame
            gp.pose = self._pose(r.grasp, r.grasp_yaw)
            self.grasp_pub.publish(gp)
            pa = PoseArray()
            pa.header = gp.header
            pa.poses = [self._pose(r.pre_grasp, r.grasp_yaw),
                        self._pose(r.grasp, r.grasp_yaw),
                        self._pose(r.lift, r.grasp_yaw)]
            self.path_pub.publish(pa)


class DetectBasket(State):
    """YASMIN state. Takes no node: grabs the shared yasmin node and runs
    perception on demand. Add it like your other states: DetectBasket().

    Camera topics default to the Gazebo sim (/head_front_camera/...). Override
    per-call (DetectBasket(depth_topic='/xtion/depth/image_raw')) or via ROS
    params basket.depth_topic / basket.rgb_topic / basket.info_topic.
    """

    def __init__(self, depth_topic=None, rgb_topic=None, info_topic=None, **cfg):
        super().__init__(outcomes=['grasp_ready', 'empty', 'no_basket', 'failed'])
        from yasmin_ros.yasmin_node import YasminNode
        node = YasminNode.get_instance()

        def _p(name, default):
            if not node.has_parameter(name):
                node.declare_parameter(name, default)
            return node.get_parameter(name).value

        dt = depth_topic or _p('basket.depth_topic', '/head_front_camera/depth/image_raw')
        rt = rgb_topic or _p('basket.rgb_topic', '/head_front_camera/rgb/image_raw')
        it = info_topic or _p('basket.info_topic', '/head_front_camera/rgb/camera_info')
        self.perception = BasketPerception(
            node, depth_topic=dt, rgb_topic=rt, info_topic=it, **cfg)

    def execute(self, blackboard):
        r = self.perception.detect_once()
        self.perception.publish(r)
        self.perception.node.get_logger().info(f'[DetectBasket] {r.status} ({r.reason})')
        if r.status == 'grasp_ready':
            blackboard['basket_pose'] = (r.basket_xy[0], r.basket_xy[1], r.rim_z)
            blackboard['basket_yaw'] = r.basket_yaw
            blackboard['pre_grasp_pose'] = r.pre_grasp
            blackboard['grasp_pose'] = r.grasp
            blackboard['lift_pose'] = r.lift
            blackboard['grasp_yaw'] = r.grasp_yaw
        return r.status