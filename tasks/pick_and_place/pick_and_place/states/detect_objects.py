import time
import numpy as np

import yasmin
import yasmin_ros

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration as ROS2Duration
from rclpy.time import Time as ROS2Time
from rclpy.action import ActionClient

import tf2_ros
from tf2_geometry_msgs import do_transform_point
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

from lasr_vision_interfaces.srv import OpenVocabDetect
from lasr_vision_interfaces.msg import Detection3D

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg


class DetectObjects(yasmin.State):
    """
    Detects groceries on the table using OPEN-VOCABULARY detection
    (lasr_vision_open_vocabulary) instead of closed COCO-YOLO.

    1. Tilt head down so the camera sees the table.
    2. Call open_vocab/detect with configured grocery queries + low thresholds.
    3. Clean labels: map returned phrase ("cup box") → matching query ("cup").
    4. Class-agnostic NMS: drop overlapping duplicates (kills stacked boxes).
    5. Project each kept box centre → 3D via depth + TF (for manipulation later).

    ROS 2 params:
        pick_and_place.objects — query words (COMMON NOUNS). Empty → default.

    Blackboard outputs:
        detected_objects : List[Detection3D]
    """

    HEAD_PAN_JOINT = "head_1_joint"
    HEAD_TILT_JOINT = "head_2_joint"
    HEAD_TILT_DOWN = -0.65

    RGB_TOPIC = "/head_front_camera/rgb/image_raw"
    DEPTH_TOPIC = "/head_front_camera/depth/image_raw"
    INFO_TOPIC = "/head_front_camera/rgb/camera_info"

    DEFAULT_QUERIES = ["cup", "can", "bottle", "bowl", "box", "iced tea", "apple"]
    BOX_THRESHOLD = 0.25
    TEXT_THRESHOLD = 0.10
    NMS_IOU = 0.5

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_output_key("detected_objects")

        self.node = yasmin_ros.logger_node
        self.bridge = CvBridge()

        try:
            q = list(
                self.node.get_parameter("pick_and_place.objects")
                .get_parameter_value()
                .string_array_value
            )
            self._queries = q or list(self.DEFAULT_QUERIES)
        except Exception:
            self._queries = list(self.DEFAULT_QUERIES)

        self._rgb = None
        self._depth = None
        self._info = None
        cam_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.node.create_subscription(Image, self.RGB_TOPIC, self._rgb_cb, cam_qos)
        self.node.create_subscription(Image, self.DEPTH_TOPIC, self._depth_cb, cam_qos)
        self.node.create_subscription(
            CameraInfo, self.INFO_TOPIC, self._info_cb, cam_qos
        )

        self._tf = tf2_ros.Buffer(cache_time=ROS2Duration(seconds=30))
        self._tf_listener = tf2_ros.TransformListener(self._tf, self.node)

        self._ovd = self.node.create_client(OpenVocabDetect, "open_vocab/detect")
        self._head = ActionClient(
            self.node, FollowJointTrajectory,
            "/head_controller/follow_joint_trajectory",
        )

    # ── camera callbacks ──
    def _rgb_cb(self, m):
        self._rgb = m

    def _depth_cb(self, m):
        self._depth = m

    def _info_cb(self, m):
        self._info = m

    # ── head ──
    def _look_down(self):
        if not self._head.wait_for_server(timeout_sec=5.0):
            yasmin.YASMIN_LOG_WARN("head controller unavailable; skipping look-down")
            return
        pt = JointTrajectoryPoint()
        pt.positions = [0.0, self.HEAD_TILT_DOWN]
        pt.time_from_start = DurationMsg(sec=2)
        traj = JointTrajectory()
        traj.joint_names = [self.HEAD_PAN_JOINT, self.HEAD_TILT_JOINT]
        traj.points = [pt]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self._head.send_goal_async(goal)
        yasmin.YASMIN_LOG_INFO("Tilting head down to look at the table…")
        time.sleep(3.0)

    # ── helpers ──
    @staticmethod
    def _wait_future(future, timeout=30.0):
        start = time.time()
        while not future.done():
            if time.time() - start > timeout:
                return None
            time.sleep(0.05)
        try:
            return future.result()
        except Exception:
            return None

    def _clean_label(self, phrase):
        p = phrase.lower()
        for q in self._queries:
            if q.lower() in p:
                return q
        return phrase

    @staticmethod
    def _iou(a, b):  # a,b = (cx,cy,w,h) midpoint format
        ax1, ay1, ax2, ay2 = a[0]-a[2]/2, a[1]-a[3]/2, a[0]+a[2]/2, a[1]+a[3]/2
        bx1, by1, bx2, by2 = b[0]-b[2]/2, b[1]-b[3]/2, b[0]+b[2]/2, b[1]+b[3]/2
        iw = max(0.0, min(ax2, bx2) - max(ax1, bx1))
        ih = max(0.0, min(ay2, by2) - max(ay1, by1))
        inter = iw * ih
        union = a[2]*a[3] + b[2]*b[3] - inter
        return inter / union if union > 0 else 0.0

    def _nms(self, dets):  # class-agnostic, keep highest-confidence per region
        kept = []
        for d in sorted(dets, key=lambda x: x[1], reverse=True):
            if all(self._iou(d[2], k[2]) < self.NMS_IOU for k in kept):
                kept.append(d)
        return kept

    def _project_3d(self, cx, cy):
        if self._depth is None or self._info is None or self._rgb is None:
            return None
        try:
            depth_img = self.bridge.imgmsg_to_cv2(self._depth, "32FC1")
        except Exception:
            return None
        h, w = depth_img.shape[:2]
        px = int(np.clip(cx, 0, w - 1))
        py = int(np.clip(cy, 0, h - 1))
        d = float(depth_img[py, px])
        if d <= 0.0 or np.isnan(d):
            return None
        K = self._info.k
        fx, fy, cxp, cyp = K[0], K[4], K[2], K[5]
        cam_frame = self._rgb.header.frame_id
        ps = PointStamped()
        ps.header.frame_id = cam_frame
        ps.header.stamp = self._rgb.header.stamp
        ps.point.x = (px - cxp) * d / fx
        ps.point.y = (py - cyp) * d / fy
        ps.point.z = d
        try:
            tr = self._tf.lookup_transform(
                "map", cam_frame, self._rgb.header.stamp,
                timeout=ROS2Duration(seconds=0.5),
            )
        except Exception:
            try:
                tr = self._tf.lookup_transform(
                    "map", cam_frame, ROS2Time(seconds=0),
                    timeout=ROS2Duration(seconds=0.5),
                )
            except Exception:
                return None
        try:
            return do_transform_point(ps, tr).point
        except Exception:
            return None

    # ── main ──
    def execute(self, blackboard):
        self._look_down()

        t0 = time.time()
        while (self._rgb is None or self._info is None) and time.time() - t0 < 5.0:
            time.sleep(0.1)
        if self._rgb is None or self._info is None:
            yasmin.YASMIN_LOG_ERROR("No camera image/info available.")
            return "failed"

        if not self._ovd.wait_for_service(timeout_sec=10.0):
            yasmin.YASMIN_LOG_ERROR("open_vocab/detect service not available.")
            return "failed"

        req = OpenVocabDetect.Request()
        req.image = self._rgb
        req.queries = list(self._queries)
        req.box_threshold = float(self.BOX_THRESHOLD)
        req.text_threshold = float(self.TEXT_THRESHOLD)
        yasmin.YASMIN_LOG_INFO(f"open_vocab queries: {self._queries}")

        resp = self._wait_future(self._ovd.call_async(req), timeout=30.0)
        if resp is None:
            yasmin.YASMIN_LOG_ERROR("open_vocab/detect failed or timed out.")
            return "failed"

        raw = [
            (d.name, float(d.confidence), (d.xywh[0], d.xywh[1], d.xywh[2], d.xywh[3]))
            for d in resp.detections
            if len(d.xywh) >= 4
        ]
        yasmin.YASMIN_LOG_INFO(f"Raw open-vocab detections ({len(raw)}):")
        for n, c, b in raw:
            yasmin.YASMIN_LOG_INFO(f"   {n}: {c:.2f} cxywh={b}")

        cleaned = [(self._clean_label(n), c, b) for n, c, b in raw]
        kept = self._nms(cleaned)

        detected = []
        for name, conf, (cx, cy, w, h) in kept:
            d3 = Detection3D()
            d3.name = name
            d3.confidence = float(conf)
            d3.xywh = [int(cx - w / 2), int(cy - h / 2), int(w), int(h)]
            pt = self._project_3d(cx, cy)
            if pt is not None:
                d3.point = pt
            detected.append(d3)

        blackboard["detected_objects"] = detected
        yasmin.YASMIN_LOG_INFO(
            f"Detected {len(detected)} object(s): "
            f"{[(d.name, round(d.confidence, 2)) for d in detected]}"
        )
        return "succeeded" if detected else "failed"