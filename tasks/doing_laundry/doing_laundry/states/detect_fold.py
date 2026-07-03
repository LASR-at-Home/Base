"""
detect_fold.py — perceive a flat T-shirt on a (white) table via COLOR, compute
a 3-step fold plan. Color mask separates cloth from the white table (works for
slightly-off-white cloth with a tight table threshold). Depth is used only to
lift the mask into 3D (grasp heights), so 1mm-thin cloth still works.

Fold plan (3 steps):
  1 left sleeve tip  -> left inner (shoulder/neck edge)
  2 right sleeve tip -> right inner
  3 orange grasp (midpoint of inner_l/inner_r) -> hem (lift & half-fold)

Markers on /fold/markers:
  red   = left  (sleeve tip + inner)
  green = right (sleeve tip + inner)
  orange= fold-3 grasp
  white = collar->hem fold line
"""

import math
import time
from dataclasses import dataclass, field

import numpy as np
import cv2
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import tf2_ros

try:
    from yasmin import State
except ImportError:
    class State:
        def __init__(self, outcomes):
            self._outcomes = outcomes

DEPTH_MIN, DEPTH_MAX = 0.3, 3.0
# table = white: tight so slightly-off-white cloth is NOT swallowed as table
TABLE_S_MAX = 25      # HSV saturation below this ...
TABLE_V_MIN = 230     # ... and value above this = table (background)
MIN_CLOTH_PX = 800


@dataclass
class FoldResult:
    status: str
    reason: str = ''
    frame: str = 'base_footprint'
    center: tuple = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    kp: dict = field(default_factory=dict)
    folds: list = field(default_factory=list)


class FoldPerception:
    def __init__(self, node, table_z=None,
                 depth_topic='/head_front_camera/depth/image_raw',
                 rgb_topic='/head_front_camera/rgb/image_raw',
                 info_topic='/head_front_camera/rgb/camera_info'):
        self.node = node
        self.table_z = table_z
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
        node.create_subscription(Image, depth_topic, self._depth_cb, qos_profile_sensor_data)
        node.create_subscription(Image, rgb_topic, self._rgb_cb, qos_profile_sensor_data)
        node.create_subscription(CameraInfo, info_topic, self._cam_cb, qos_profile_sensor_data)
        self.mpub = node.create_publisher(MarkerArray, '/fold/markers', 10)

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
                self.tf_buffer.lookup_transform(cand, cam, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5))
                self.base = cand
                return cand
            except Exception:
                continue
        return None

    def _tf(self, base, cam):
        for q in ([self.depth_stamp] if self.depth_stamp else []) + [rclpy.time.Time()]:
            try:
                return self.tf_buffer.lookup_transform(base, cam, q,
                    timeout=rclpy.duration.Duration(seconds=0.3))
            except Exception:
                continue
        return None

    def _px_to_base(self, us, vs, base, cam):
        """back-project pixel (u,v) with its depth to base frame xyz."""
        zs = self.depth[vs, us]
        ok = (zs >= DEPTH_MIN) & (zs <= DEPTH_MAX) & np.isfinite(zs)
        us, vs, zs = us[ok], vs[ok], zs[ok]
        if zs.size == 0:
            return None
        xc = (us - self.cx) * zs / self.fx
        yc = (vs - self.cy) * zs / self.fy
        tr = self._tf(base, cam)
        if tr is None:
            return None
        q, t = tr.transform.rotation, tr.transform.translation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        R = np.array([
            [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
            [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
            [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]])
        return np.stack([xc, yc, zs], 1) @ R.T + np.array([t.x, t.y, t.z])

    def detect_once(self, timeout=2.0):
        t0 = time.time()
        while self.depth is None or self.rgb is None or self.fx is None:
            if time.time() - t0 > timeout:
                return FoldResult('failed', 'no depth/rgb/intrinsics')
            time.sleep(0.05)
        cam = self.depth_frame
        base = self._base_frame(cam)
        if base is None:
            return FoldResult('failed', 'no tf')

        rgb = self.rgb
        if rgb.shape[:2] != self.depth.shape[:2]:
            rgb = cv2.resize(rgb, (self.depth.shape[1], self.depth.shape[0]))
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        k = 40
        corners = [hsv[:k, :k], hsv[:k, -k:], hsv[-k:, :k], hsv[-k:, -k:]]
        cand = [np.median(c.reshape(-1, 3), axis=0) for c in corners]
        table_hsv = min(cand, key=lambda c: c[1] - c[2])   # whitest corner = table
        S, V = hsv[:, :, 1], hsv[:, :, 2]
        white = (S < table_hsv[1] + 20) & (V > table_hsv[2] - 30)
        cloth = (~white).astype(np.uint8)
        cloth = cv2.morphologyEx(cloth, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        cloth = cv2.morphologyEx(cloth, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))

        cnts, _ = cv2.findContours(cloth, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return FoldResult('failed', 'no cloth contour', frame=base)
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) < MIN_CLOTH_PX:
            return FoldResult('failed', 'cloth too small', frame=base)

        # pixel mask -> base-frame points for center/axis/z
        ys, xs = np.where(cloth > 0)
        pts = self._px_to_base(xs, ys, base, cam)
        if pts is None or pts.shape[0] < 50:
            return FoldResult('failed', 'no depth under mask', frame=base)
        cen = pts[:, :2].mean(0)
        z_shirt = float(np.median(pts[:, 2]))

        # PCA on the base-frame cloth points -> body axis
        rel2 = pts[:, :2] - cen
        cov = (rel2.T @ rel2) / rel2.shape[0]
        w, v = np.linalg.eigh(cov)
        # major axis = longest cloth extent; for a T-shirt that's the SLEEVE line
        e0, e1 = v[:, 0], v[:, 1]
        body_ax = e0 if abs(e0[0]) >= abs(e1[0]) else e1   # more x-aligned = collar->hem
        sleeve_ax = e1 if abs(e0[0]) >= abs(e1[0]) else e0
        # orient body_ax so +points toward collar (robot-far, larger x)
        if body_ax[0] < 0:
            body_ax = -body_ax
        sl = sleeve_ax / np.linalg.norm(sleeve_ax)
        bo = body_ax / np.linalg.norm(body_ax)

        along = rel2 @ bo      # collar(+)..hem(-)
        lat = rel2 @ sl        # sleeve +/-

        def P(a, l):
            xy = cen + a * bo + l * sl
            return (float(xy[0]), float(xy[1]), z_shirt)

        a_top = float(np.percentile(along, 95))    # collar
        a_hem = float(np.percentile(along, 5))     # hem
        l_pos = float(np.percentile(lat, 95))      # one sleeve
        l_neg = float(np.percentile(lat, 5))       # other sleeve
        a_sh = a_top * 0.5                         # shoulder line (between collar & center)

        # left = +lat side, right = -lat side
        sleeve_l = P(a_sh, l_pos)
        sleeve_r = P(a_sh, l_neg)
        inner_l = P(a_sh, l_pos - (l_pos - l_neg) * (2/3))
        inner_r = P(a_sh, l_neg - (l_neg - l_pos) * (2/3))
        top = P(a_top, 0.0)
        hem = P(a_hem, 0.0)
        mid = ((inner_l[0]+inner_r[0])/2, (inner_l[1]+inner_r[1])/2, z_shirt)

        kp = {'sleeve_l': sleeve_l, 'sleeve_r': sleeve_r,
              'inner_l': inner_l, 'inner_r': inner_r, 'top': top, 'hem': hem}
        folds = [
            (sleeve_l, inner_l, 'fold_sleeve_left'),
            (sleeve_r, inner_r, 'fold_sleeve_right'),
            (mid, hem, 'lift_and_fold'),
        ]
        return FoldResult('fold_ready', 'ok', frame=base,
                          center=(float(cen[0]), float(cen[1]), z_shirt),
                          yaw=math.atan2(bo[1], bo[0]), kp=kp, folds=folds)

    def _sphere(self, xyz, rgb, ns, i, frame, now, s=0.03):
        m = Marker()
        m.header.frame_id = frame; m.header.stamp = now
        m.ns = ns; m.id = i
        m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = s
        m.color.r, m.color.g, m.color.b, m.color.a = float(rgb[0]), float(rgb[1]), float(rgb[2]), 1.0
        return m

    def publish_markers(self, r: FoldResult):
        arr = MarkerArray()
        now = self.node.get_clock().now().to_msg()
        kpc = {'sleeve_l': (1,0,0), 'inner_l': (1,0,0),
               'sleeve_r': (0,1,0), 'inner_r': (0,1,0),
               'top': (0.6,0.6,0.6), 'hem': (0.6,0.6,0.6)}
        i = 0
        for name, xyz in r.kp.items():
            arr.markers.append(self._sphere(xyz, kpc.get(name, (0.6,0.6,0.6)), 'kp', i, r.frame, now, 0.028)); i += 1
        # fold-3 orange grasp
        if r.folds:
            arr.markers.append(self._sphere(r.folds[2][0], (1,0.5,0), 'grasp3', i, r.frame, now, 0.032)); i += 1
        # move lines (yellow)
        for j, (g, p, kind) in enumerate(r.folds):
            ln = Marker()
            ln.header.frame_id = r.frame; ln.header.stamp = now
            ln.ns = 'move'; ln.id = 200 + j
            ln.type = Marker.LINE_STRIP; ln.action = Marker.ADD
            ln.scale.x = 0.006
            ln.color.r = ln.color.g = ln.color.a = 1.0
            ln.pose.orientation.w = 1.0
            ln.points = [Point(x=g[0], y=g[1], z=g[2]), Point(x=p[0], y=p[1], z=p[2])]
            arr.markers.append(ln)
        # fold line (white, collar->hem)
        line = Marker()
        line.header.frame_id = r.frame; line.header.stamp = now
        line.ns = 'fold_line'; line.id = 100
        line.type = Marker.LINE_STRIP; line.action = Marker.ADD
        line.scale.x = 0.008
        line.color.r = line.color.g = line.color.b = line.color.a = 1.0
        line.pose.orientation.w = 1.0
        line.points = [Point(x=r.kp['top'][0], y=r.kp['top'][1], z=r.kp['top'][2]),
                       Point(x=r.kp['hem'][0], y=r.kp['hem'][1], z=r.kp['hem'][2])]
        arr.markers.append(line)
        self.mpub.publish(arr)


class DetectFold(State):
    def __init__(self, table_z=None, depth_topic=None, rgb_topic=None, info_topic=None):
        super().__init__(outcomes=['fold_ready', 'failed'])
        from yasmin_ros.yasmin_node import YasminNode
        node = YasminNode.get_instance()
        self.per = FoldPerception(
            node, table_z=table_z,
            depth_topic=depth_topic or '/head_front_camera/depth/image_raw',
            rgb_topic=rgb_topic or '/head_front_camera/rgb/image_raw',
            info_topic=info_topic or '/head_front_camera/rgb/camera_info')

    def execute(self, blackboard):
        r = self.per.detect_once()
        if r.status == 'fold_ready':
            self.per.publish_markers(r)
        self.per.node.get_logger().info(f'[DetectFold] {r.status} ({r.reason})')
        if r.status == 'fold_ready':
            blackboard['fold_center'] = r.center
            blackboard['fold_yaw'] = r.yaw
            blackboard['fold_kp'] = r.kp
            blackboard['fold_plan'] = r.folds
        return r.status