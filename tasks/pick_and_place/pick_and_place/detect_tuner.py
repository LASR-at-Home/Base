#!/usr/bin/env python3
"""Interactive open-vocab DETECTION TUNER. Park in front of the table, run, watch
RViz Image /detect_tuner/image + the per-detection table. Tune live with ros2 param set."""
import math
import time

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration as RDuration
from rclpy.time import Time as RTime

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import tf2_ros
from tf2_geometry_msgs import do_transform_point
from cv_bridge import CvBridge

from lasr_vision_interfaces.srv import OpenVocabDetect


class DetectTuner(Node):
    def __init__(self):
        super().__init__("detect_tuner")
        self.declare_parameter("rgb_topic", "/head_front_camera/rgb/image_raw")
        self.declare_parameter("depth_topic", "/head_front_camera/depth/image_raw")
        self.declare_parameter("info_topic", "/head_front_camera/rgb/camera_info")
        self.declare_parameter("queries", ["cup", "can", "bottle", "box", "apple"])
        self.declare_parameter("box_threshold", 0.25)
        self.declare_parameter("text_threshold", 0.20)
        self.declare_parameter("period", 3.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("head_tilt", -0.6)   # rad; 99.0 to skip
        self.declare_parameter("tilt_once", True)

        self.bridge = CvBridge()
        self._rgb = None
        self._depth = None
        self._info = None

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        rgb_t = self.get_parameter("rgb_topic").value
        depth_t = self.get_parameter("depth_topic").value
        info_t = self.get_parameter("info_topic").value
        self.create_subscription(Image, rgb_t, self._rgb_cb, qos)
        self.create_subscription(Image, depth_t, self._depth_cb, qos)
        self.create_subscription(CameraInfo, info_t, self._info_cb, qos)

        self._pub = self.create_publisher(Image, "/detect_tuner/image", 10)
        self._ovd = self.create_client(OpenVocabDetect, "open_vocab/detect")
        self._tf = tf2_ros.Buffer()
        self._tfl = tf2_ros.TransformListener(self._tf, self)
        self._head = ActionClient(
            self, FollowJointTrajectory, "/head_controller/follow_joint_trajectory")
        self.get_logger().info(
            f"detect_tuner up. RGB={rgb_t}\n"
            f"  -> RViz: add Image display on /detect_tuner/image\n"
            f"  -> tune live: ros2 param set /detect_tuner box_threshold 0.35")

    def _rgb_cb(self, m):
        self._rgb = m

    def _depth_cb(self, m):
        self._depth = m

    def _info_cb(self, m):
        self._info = m

    def _spin(self, secs):
        end = time.time() + secs
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_future(self, fut, secs):
        end = time.time() + secs
        while not fut.done() and time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.result() if fut.done() else None

    def tilt_head(self, tilt):
        if not self._head.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("head controller not available — skipping tilt")
            return
        g = FollowJointTrajectory.Goal()
        t = JointTrajectory()
        t.joint_names = ["head_1_joint", "head_2_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [0.0, float(tilt)]
        pt.time_from_start = Duration(sec=2)
        t.points = [pt]
        g.trajectory = t
        self._head.send_goal_async(g)
        self.get_logger().info(f"tilting head to {tilt:.2f} rad")
        self._spin(3.0)

    def _project(self, cx, cy):
        if self._depth is None or self._info is None or self._rgb is None:
            return None, None
        enc = self._depth.encoding
        depth = self.bridge.imgmsg_to_cv2(self._depth, enc)
        scale = 0.001 if enc in ("16UC1", "mono16") else 1.0
        h, w = depth.shape[:2]
        px = int(np.clip(cx, 0, w - 1))
        py = int(np.clip(cy, 0, h - 1))
        d = float(depth[py, px]) * scale
        if d <= 0.0 or math.isnan(d) or d > 8.0:
            return d, None
        K = self._info.k
        fx, fy, cxp, cyp = K[0], K[4], K[2], K[5]
        cam = self._rgb.header.frame_id
        ps = PointStamped()
        ps.header.frame_id = cam
        ps.header.stamp = self._rgb.header.stamp
        ps.point.x = (px - cxp) * d / fx
        ps.point.y = (py - cyp) * d / fy
        ps.point.z = d
        map_frame = self.get_parameter("map_frame").value
        try:
            tr = self._tf.lookup_transform(map_frame, cam, RTime(),
                                           timeout=RDuration(seconds=0.5))
            mp = do_transform_point(ps, tr).point
            return d, (mp.x, mp.y, mp.z)
        except Exception:
            return d, None

    @staticmethod
    def _color(conf):
        if conf >= 0.45:
            return (0, 255, 0)
        if conf >= 0.30:
            return (0, 255, 255)
        return (0, 0, 255)

    def detect_once(self):
        if self._rgb is None or self._info is None:
            self.get_logger().warn("waiting for camera image/info…")
            return
        if not self._ovd.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("open_vocab/detect not available — is the node up?")
            return
        queries = list(self.get_parameter("queries").value)
        box_thr = float(self.get_parameter("box_threshold").value)
        text_thr = float(self.get_parameter("text_threshold").value)
        req = OpenVocabDetect.Request()
        req.image = self._rgb
        req.queries = queries
        req.box_threshold = box_thr
        req.text_threshold = text_thr
        resp = self._wait_future(self._ovd.call_async(req), 120.0)
        if resp is None:
            self.get_logger().error("detect timed out (CPU slow? disable clip_rerank)")
            return
        dets = []
        for d in resp.detections:
            if len(d.xywh) < 4:
                continue
            cx, cy, w, h = (float(d.xywh[0]), float(d.xywh[1]),
                            float(d.xywh[2]), float(d.xywh[3]))
            depth, mapxyz = self._project(cx, cy)
            dets.append((d.name, float(d.confidence), (cx, cy, w, h), depth, mapxyz))
        dets.sort(key=lambda x: x[1], reverse=True)
        self.get_logger().info(
            f"\n=== {len(dets)} detection(s)  queries={queries}  "
            f"box_thr={box_thr}  text_thr={text_thr} ===")
        for i, (name, conf, (cx, cy, w, h), depth, mapxyz) in enumerate(dets):
            dstr = f"{depth:.2f}m" if depth is not None else "n/a"
            mstr = (f"map=({mapxyz[0]:.2f},{mapxyz[1]:.2f},{mapxyz[2]:.2f})"
                    if mapxyz is not None else "map=n/a")
            self.get_logger().info(
                f"  [{i}] {name:<14} conf={conf:.2f}  "
                f"cxywh=({cx:.0f},{cy:.0f},{w:.0f},{h:.0f})  depth={dstr}  {mstr}")
        try:
            img = self.bridge.imgmsg_to_cv2(self._rgb, "bgr8")
            for name, conf, (cx, cy, w, h), depth, _ in dets:
                x1, y1 = int(cx - w / 2), int(cy - h / 2)
                x2, y2 = int(cx + w / 2), int(cy + h / 2)
                col = self._color(conf)
                cv2.rectangle(img, (x1, y1), (x2, y2), col, 2)
                dstr = f" {depth:.2f}m" if depth is not None else ""
                cv2.putText(img, f"{name} {conf:.2f}{dstr}", (x1, max(0, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
            out = self.bridge.cv2_to_imgmsg(img, "bgr8")
            out.header = self._rgb.header
            self._pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f"annotate/publish failed: {e}")

    def run(self):
        if (self.get_parameter("tilt_once").value
                and abs(float(self.get_parameter("head_tilt").value)) < 1.6):
            self._spin(1.0)
            self.tilt_head(self.get_parameter("head_tilt").value)
        while rclpy.ok():
            self.detect_once()
            self._spin(max(0.5, float(self.get_parameter("period").value)))


def main():
    rclpy.init()
    node = DetectTuner()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()