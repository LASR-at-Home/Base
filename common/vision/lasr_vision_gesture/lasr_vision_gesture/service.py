#!/usr/bin/env python3
"""Gesture detection service.

People are classified on the 2D pose keypoints from ``/yolo/detect_pose`` so the
same code works with only an RGB camera. With ``use_3d`` the same frame is also
sent to ``/yolo/detect3d_pose``; both YOLO services return people in the same
order, so person ``i`` of the 2D response is person ``i`` of the 3D response and
the 3D response is only used for the location.
"""

import threading
import time
from typing import Dict, List, Optional, Tuple

import message_filters
import rclpy
from geometry_msgs.msg import PointStamped
from lasr_vision_interfaces.srv import (
    DetectGesture,
    YoloPoseDetection,
    YoloPoseDetection3D,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from .classifiers import ALL_GESTURES, DYNAMIC, STATIC
from .classifiers.geometry import TORSO, person_location

MODEL = "yolo11n-pose.pt"
CONFIDENCE = 0.5
YOLO_TIMEOUT = 5.0  # seconds to wait for a single YOLO response
DEPTH_WAIT = 2.0  # seconds to wait for depth before falling back to 2D


class GestureService(Node):
    def __init__(self):
        super().__init__("gesture_service")
        self.declare_parameter("rgb_topic", "/head_front_camera/rgb/image_raw")
        self.declare_parameter("depth_topic", "/head_front_camera/depth/image_raw")
        self.declare_parameter(
            "camera_info_topic", "/head_front_camera/depth/camera_info"
        )
        rgb_topic = self.get_parameter("rgb_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        info_topic = self.get_parameter("camera_info_topic").value

        self._cb_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._rgb: Optional[Image] = None
        self._rgbd: Optional[Tuple[Image, Image, CameraInfo]] = None

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        # plain subscription so 2D mode works without a depth camera
        self.create_subscription(
            Image, rgb_topic, self._rgb_cb, qos, callback_group=self._cb_group
        )
        rgb_sub = message_filters.Subscriber(
            self, Image, rgb_topic, qos_profile=qos, callback_group=self._cb_group
        )
        depth_sub = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=qos, callback_group=self._cb_group
        )
        info_sub = message_filters.Subscriber(
            self, CameraInfo, info_topic, qos_profile=qos, callback_group=self._cb_group
        )
        self._info_cache = message_filters.Cache(info_sub, 10)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=1.0
        )
        self._sync.registerCallback(self._rgbd_cb)

        self._pose_2d = self.create_client(
            YoloPoseDetection, "/yolo/detect_pose", callback_group=self._cb_group
        )
        self._pose_3d = self.create_client(
            YoloPoseDetection3D, "/yolo/detect3d_pose", callback_group=self._cb_group
        )
        self.create_service(
            DetectGesture,
            "/gesture/detect",
            self._handle,
            callback_group=self._cb_group,
        )
        self.get_logger().info(f"Gesture service ready, gestures: {ALL_GESTURES}")

    # -- camera ---------------------------------------------------------

    def _rgb_cb(self, msg: Image):
        with self._lock:
            self._rgb = msg

    def _rgbd_cb(self, rgb: Image, depth: Image):
        info = self._info_cache.getLast()
        if info is None:
            return
        with self._lock:
            self._rgbd = (rgb, depth, info)

    def _latest(self, use_3d: bool):
        with self._lock:
            return self._rgbd if use_3d else self._rgb

    # -- yolo -----------------------------------------------------------

    def _call(self, client, request):
        """Call a service without blocking the executor thread's spin."""
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(YOLO_TIMEOUT):
            self.get_logger().warn(f"{client.srv_name} did not respond in time")
            return None
        return future.result()

    def _detect_2d(self, image: Image) -> Optional[List[Dict]]:
        req = YoloPoseDetection.Request()
        req.image_raw = image
        req.model = MODEL
        req.confidence = CONFIDENCE
        res = self._call(self._pose_2d, req)
        if res is None:
            return None
        # YOLO reports undetected keypoints at (0, 0)
        return [
            {
                k.keypoint_name: (float(k.x), float(k.y))
                for k in det.keypoints
                if (k.x, k.y) != (0, 0)
            }
            for det in res.detections
        ]

    def _detect_3d(self, rgbd, target_frame: str) -> Optional[List[Dict]]:
        image, depth, info = rgbd
        req = YoloPoseDetection3D.Request()
        req.image_raw = image
        req.depth_image = depth
        req.depth_camera_info = info
        req.model = MODEL
        req.confidence = CONFIDENCE
        req.target_frame = target_frame
        res = self._call(self._pose_3d, req)
        if res is None:
            return None
        return [
            {
                k.keypoint_name: (k.point.x, k.point.y, k.point.z)
                for k in det.keypoints
            }
            for det in res.detections
        ]

    @staticmethod
    def _location_3d(kp3: Dict) -> Optional[Tuple[float, float, float]]:
        """Shoulder midpoint, else torso, else mean of anything that has depth."""
        for names in (("left_shoulder", "right_shoulder"), TORSO, tuple(kp3)):
            pts = [kp3[n] for n in names if n in kp3]
            if pts:
                return tuple(sum(p[i] for p in pts) / len(pts) for i in range(3))
        return None

    # -- service --------------------------------------------------------

    def _handle(self, req: DetectGesture.Request, res: DetectGesture.Response):
        gesture = req.gesture
        if gesture not in STATIC and gesture not in DYNAMIC:
            self.get_logger().error(
                f"Unknown gesture '{gesture}', expected one of {ALL_GESTURES}"
            )
            return res

        use_3d = req.use_3d
        if use_3d:
            deadline = time.monotonic() + DEPTH_WAIT
            while self._latest(True) is None and time.monotonic() < deadline:
                time.sleep(0.1)
            if self._latest(True) is None:
                self.get_logger().warn(
                    "No depth received, falling back to 2D. Locations will be "
                    "in pixels with z = 0 and frame 'image'"
                )
                use_3d = False
        else:
            self.get_logger().warn(
                "use_3d is false: locations are in pixels with z = 0 (frame 'image')"
            )
        target_frame = req.target_frame or "map"

        wave = DYNAMIC[gesture]() if gesture in DYNAMIC else None
        timeout = req.timeout if req.timeout > 0 else 5.0
        deadline = time.monotonic() + timeout
        last_stamp = None

        while time.monotonic() < deadline and rclpy.ok():
            frame_in = self._latest(use_3d)
            if frame_in is None:
                time.sleep(0.05)
                continue
            image = frame_in[0] if use_3d else frame_in
            stamp = (image.header.stamp.sec, image.header.stamp.nanosec)
            if stamp == last_stamp:
                time.sleep(0.02)
                continue
            last_stamp = stamp

            people = self._detect_2d(image)
            if people is None:
                continue
            people_3d = None
            if use_3d:
                people_3d = self._detect_3d(frame_in, target_frame)
                if people_3d is None:
                    continue
                if len(people_3d) != len(people):
                    self.get_logger().warn(
                        f"2D/3D person count differ ({len(people)} vs "
                        f"{len(people_3d)}), skipping frame"
                    )
                    continue

            if wave is not None:
                matches = wave.update(people)
            else:
                matches = [i for i, kp in enumerate(people) if STATIC[gesture](kp)]
            if not matches:
                continue

            locations = []
            for i in matches:
                point = PointStamped()
                if use_3d:
                    loc = self._location_3d(people_3d[i])
                    if loc is None:
                        self.get_logger().warn(f"Person {i} has no depth, dropping")
                        continue
                    point.header.frame_id = target_frame
                    point.header.stamp = frame_in[1].header.stamp
                    point.point.x, point.point.y, point.point.z = loc
                else:
                    loc = person_location(people[i])
                    if loc is None:
                        continue
                    point.header.frame_id = "image"
                    point.header.stamp = image.header.stamp
                    point.point.x, point.point.y = loc
                locations.append(point)
            if locations:
                res.locations = locations
                self.get_logger().info(
                    f"Found {len(locations)} person(s) making '{gesture}'"
                )
                return res

        self.get_logger().info(f"No one making '{gesture}' within {timeout:.1f}s")
        return res


def main(args=None):
    rclpy.init(args=args)
    node = GestureService()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
