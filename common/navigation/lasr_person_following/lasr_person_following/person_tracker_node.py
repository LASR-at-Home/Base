"""
person_tracker_node: continuously detects and tracks people, and keeps track of the host.

Pipeline (runs all the time, independent of any state machine):

    synced RGB + depth --> /yolo/detect3d (async, one request in flight, latest frame)
        --> person detections in `fixed_frame` (odom) + torso colour histogram
        --> MultiTracker (Kalman per person) --> HostTracker (which one is the host)
        --> topics below; head control keeps the host in view

Topics (all host positions in `fixed_frame`):
    ~/host_pose    geometry_msgs/PoseStamped   predicted host position, yaw = walking direction
    ~/host_point   geometry_msgs/PointStamped  same position (for RViz / evaluation)
    ~/host_state   std_msgs/String             JSON: status, velocity, stationary, ... (10 Hz)
    ~/markers      visualization_msgs/MarkerArray  all tracks, host highlighted

Services:
    ~/lock_host    std_srvs/Trigger   lock onto the person in front of the robot
    ~/release_host std_srvs/Trigger   forget the host
    ~/new_leg      std_srvs/Trigger   reset "distance travelled" (host said "not arrived")
    ~/head_tracking std_srvs/SetBool  enable/disable pointing the head at the host
    ~/enable       std_srvs/SetBool   start/stop detecting (YOLO is only called while
                                      enabled; disabling also forgets all tracks)
"""

import json
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient

import message_filters
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, ColorRGBA
from std_srvs.srv import Trigger, SetBool
from visualization_msgs.msg import Marker, MarkerArray
from control_msgs.action import PointHead
from lasr_vision_interfaces.srv import YoloDetection3D

from lasr_person_following.tracking import (
    Detection,
    HostParams,
    HostStatus,
    HostTracker,
    MultiTracker,
    TrackerParams,
    TrackState,
    appearance_similarity,
)

H_BINS, S_BINS = 12, 4


def torso_histogram(bgr: np.ndarray, xywh, xyseg) -> np.ndarray:
    """L1-normalised hue/saturation histogram of the torso (upper-middle of the box),
    restricted to the segmentation mask and to pixels with enough colour/brightness."""
    x, y, w, h = xywh
    H, W = bgr.shape[:2]
    # Head cut off by the top of the image (person very close): the crop would land on a
    # different body part, so no appearance rather than a misleading one.
    if y <= 2:
        return None
    # Feet cut off at the bottom (common when following): measure the torso from the top
    # of the box, scaled by the full body height estimated from the box width.
    full_h = max(h, int(3.5 * w)) if y + h >= H - 2 else h
    x0, x1 = max(0, x + w // 4), min(W, x + 3 * w // 4)
    y0, y1 = max(0, y + int(0.2 * full_h)), min(H, y + int(0.55 * full_h))
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None
    crop = bgr[y0:y1, x0:x1]
    mask = None
    if len(xyseg) >= 6:
        poly = np.array(xyseg, dtype=np.int32).reshape(-1, 2) - np.array([x0, y0])
        mask = np.zeros(crop.shape[:2], np.uint8)
        cv2.fillPoly(mask, [poly], 255)
        if cv2.countNonZero(mask) < 30:
            mask = None
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    # hue is meaningless for dark/grey pixels: count those in a separate "grey" bin
    # by value instead
    sat_ok = cv2.inRange(hsv, (0, 40, 40), (180, 255, 255))
    if mask is not None:
        colour_mask = cv2.bitwise_and(sat_ok, mask)
        grey_mask = cv2.bitwise_and(cv2.bitwise_not(sat_ok), mask)
    else:
        colour_mask, grey_mask = sat_ok, cv2.bitwise_not(sat_ok)
    hs = cv2.calcHist([hsv], [0, 1], colour_mask, [H_BINS, S_BINS], [0, 180, 40, 256]).flatten()
    v = cv2.calcHist([hsv], [2], grey_mask, [4], [0, 256]).flatten()
    hist = np.concatenate([hs, v]).astype(np.float64)
    total = hist.sum()
    if total < 30:
        return None
    return hist / total


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class PersonTrackerNode(Node):
    def __init__(self):
        super().__init__("person_tracker")
        d = self.declare_parameter
        d("image_topic", "/head_front_camera/rgb/image_raw")
        d("depth_topic", "/head_front_camera/depth/image_raw")
        d("camera_info_topic", "/head_front_camera/depth/camera_info")
        d("yolo_service", "/yolo/detect3d")
        d("model", "yolo11n-seg.pt")
        d("confidence", 0.45)
        d("fixed_frame", "odom")
        d("robot_frame", "base_footprint")
        d("max_range", 7.0)
        d("min_centroid_z", 0.3)  # mask centroid height of a standing/sitting person
        d("max_centroid_z", 1.6)
        d("head_tracking", True)
        d("head_rate", 5.0)
        d("head_target_z", 1.1)
        d("head_deadband_deg", 4.0)
        d("head_lookahead", 0.3)  # s, aim where the host will be
        d("max_detection_rate", 15.0)
        d("lock_max_range", 3.0)
        d("start_enabled", False)
        d("surface_offset", 0.12)  # m: mask points lie on the body surface, push to centre
        d("stationary_time", 4.0)
        d("reid_relaxed", True)  # see HostParams.reid_relaxed_enabled

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.fixed_frame = p("fixed_frame")
        self.robot_frame = p("robot_frame")

        self.tracker = MultiTracker(TrackerParams())
        self.host = HostTracker(self.tracker, HostParams(
            lock_max_range=p("lock_max_range"), stationary_time=p("stationary_time"),
            reid_relaxed_enabled=p("reid_relaxed")))
        self.enabled = p("start_enabled")
        self.lock = threading.RLock()
        self.bridge = CvBridge()

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- camera input ----
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.cam_group = MutuallyExclusiveCallbackGroup()
        rgb = message_filters.Subscriber(self, Image, p("image_topic"), qos_profile=qos,
                                         callback_group=self.cam_group)
        depth = message_filters.Subscriber(self, Image, p("depth_topic"), qos_profile=qos,
                                           callback_group=self.cam_group)
        self.sync = message_filters.ApproximateTimeSynchronizer([rgb, depth], 5, 0.05)
        self.sync.registerCallback(self._frames_cb)
        self.camera_info = None
        self.create_subscription(CameraInfo, p("camera_info_topic"), self._info_cb, qos,
                                 callback_group=self.cam_group)
        self.latest = None  # (rgb, depth)
        self.latest_seq = 0
        self.sent_seq = 0

        # ---- detector ----
        self.client_group = ReentrantCallbackGroup()
        self.yolo = self.create_client(YoloDetection3D, p("yolo_service"),
                                       callback_group=self.client_group)
        self.in_flight = False
        self.det_times = []  # for rate/latency diagnostics
        self.diag = {"frames": 0, "empty": 0, "raw": 0, "nan": 0, "z": 0, "range": 0, "kept": 0}
        self.latencies = []
        self.create_timer(1.0 / p("max_detection_rate"), self._request_detection,
                          callback_group=self.client_group)

        # ---- outputs ----
        self.pose_pub = self.create_publisher(PoseStamped, "~/host_pose", 10)
        self.point_pub = self.create_publisher(PointStamped, "~/host_point", 10)
        self.state_pub = self.create_publisher(String, "~/host_state", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "~/markers", 10)
        self.out_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(0.1, self._publish, callback_group=self.out_group)
        self.create_timer(5.0, self._diagnostics, callback_group=self.out_group)

        # ---- head ----
        self.head_enabled = p("head_tracking")
        self.head_client = ActionClient(self, PointHead, "/head_controller/point_head_action",
                                        callback_group=self.client_group)
        self.last_head_target = None
        self.last_head_time = 0.0
        self.create_timer(1.0 / p("head_rate"), self._head, callback_group=self.out_group)

        # ---- services ----
        srv_group = MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger, "~/lock_host", self._lock_srv, callback_group=srv_group)
        self.create_service(Trigger, "~/release_host", self._release_srv, callback_group=srv_group)
        self.create_service(Trigger, "~/new_leg", self._new_leg_srv, callback_group=srv_group)
        self.create_service(SetBool, "~/head_tracking", self._head_srv, callback_group=srv_group)
        self.create_service(SetBool, "~/enable", self._enable_srv, callback_group=srv_group)

        self.get_logger().info("person_tracker ready")

    # ------------------------------------------------------------------ input

    def _info_cb(self, msg):
        self.camera_info = msg

    def _frames_cb(self, rgb, depth):
        self.latest = (rgb, depth)
        self.latest_seq += 1

    def _request_detection(self):
        if not self.enabled or self.in_flight or self.latest is None or self.camera_info is None:
            return
        if self.latest_seq == self.sent_seq:
            return
        if not self.yolo.service_is_ready():
            self.get_logger().warn("waiting for YOLO service", throttle_duration_sec=5.0)
            return
        rgb, depth = self.latest
        self.sent_seq = self.latest_seq
        req = YoloDetection3D.Request(
            image_raw=rgb, depth_image=depth, depth_camera_info=self.camera_info,
            model=self.get_parameter("model").value,
            confidence=float(self.get_parameter("confidence").value),
            filter=["person"], target_frame=self.fixed_frame,
        )
        self.in_flight = True
        sent = time.monotonic()
        fut = self.yolo.call_async(req)
        fut.add_done_callback(lambda f: self._detections_cb(f, rgb, sent))

    def _detections_cb(self, fut, rgb_msg, sent):
        self.in_flight = False
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().warn(f"YOLO call failed: {e}", throttle_duration_sec=5.0)
            return
        now = time.monotonic()
        self.latencies.append(now - sent)
        self.det_times.append(now)
        stamp = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9

        robot = self._robot_pose(rclpy.time.Time.from_msg(rgb_msg.header.stamp))
        try:
            bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        except Exception:
            bgr = None

        dets = []
        self.diag["frames"] += 1
        self.diag["raw"] += len(res.detected_objects)
        if not res.detected_objects:
            self.diag["empty"] += 1
        for obj in res.detected_objects:
            pt = obj.point
            if not all(math.isfinite(v) for v in (pt.x, pt.y, pt.z)):
                self.diag["nan"] += 1
                continue
            if not (self.get_parameter("min_centroid_z").value <= pt.z
                    <= self.get_parameter("max_centroid_z").value):
                self.diag["z"] += 1
                self.get_logger().info(f"rejected detection: centroid z {pt.z:.2f}", throttle_duration_sec=1.0)
                continue
            rng = 2.0
            px, py = pt.x, pt.y
            if robot is not None:
                rng = math.hypot(pt.x - robot[0], pt.y - robot[1])
                if rng > self.get_parameter("max_range").value:
                    self.diag["range"] += 1
                    continue
                off = self.get_parameter("surface_offset").value
                if rng > 1e-3:
                    px += (pt.x - robot[0]) / rng * off
                    py += (pt.y - robot[1]) / rng * off
            look = None
            if bgr is not None and len(obj.xywh) == 4:
                look = torso_histogram(bgr, obj.xywh, obj.xyseg)
            dets.append(Detection(
                x=px, y=py, stamp=stamp, confidence=obj.confidence,
                appearance=look, sigma=0.1 + 0.05 * rng,
            ))
        self.diag["kept"] += len(dets)
        with self.lock:
            matches = self.tracker.step(dets, stamp)
            host_id = self.host.host_id
            if host_id is not None and dets and not any(tr.id == host_id for tr, _ in matches):
                host = self.tracker.get(host_id)
                if host is not None:
                    self.get_logger().info(
                        f"host track {host_id} at ({host.position[0]:.2f}, {host.position[1]:.2f}) "
                        f"std {host.position_std():.2f} not matched; detections "
                        + ", ".join(f"({d.x:.2f}, {d.y:.2f}) sim {appearance_similarity(host.appearance, d.appearance):.2f}"
                                    for d in dets),
                        throttle_duration_sec=0.5)

    def _robot_pose(self, at=None):
        try:
            t = self.tf_buffer.lookup_transform(
                self.fixed_frame, self.robot_frame, at or rclpy.time.Time(),
                timeout=Duration(seconds=0.1)).transform
        except Exception:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.fixed_frame, self.robot_frame, rclpy.time.Time()).transform
            except Exception:
                return None
        return t.translation.x, t.translation.y, _yaw(t.rotation)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ output

    def _publish(self):
        now = self._now()
        with self.lock:
            # The filter stays at the last measurement time (detections arrive with
            # ~100 ms latency); outputs are extrapolated to `now` instead.
            est = self.host.update(now)
            tracks = []
            for tr in self.tracker.tracks:
                dt = max(0.0, min(now - tr.stamp, 1.0))
                pos = (tr.position[0] + tr.velocity[0] * dt, tr.position[1] + tr.velocity[1] * dt)
                tracks.append((tr.id, pos, tr.velocity, tr.state))
                if tr.id == est.track_id:
                    est.position = pos
        self.estimate = est
        state = {
            "status": est.status.value,
            "track_id": est.track_id,
            "x": est.position[0] if est.position else None,
            "y": est.position[1] if est.position else None,
            "vx": est.velocity[0], "vy": est.velocity[1],
            "speed": math.hypot(*est.velocity),
            "std": est.position_std,
            "last_seen_age": (now - est.last_seen) if est.last_seen is not None else None,
            "stationary": est.stationary,
            "still_for": est.still_for,
            "travelled": est.travelled,
            "frame": self.fixed_frame,
            "stamp": now,
        }
        self.state_pub.publish(String(data=json.dumps(state)))

        stamp = self.get_clock().now().to_msg()
        if est.position is not None:
            ps = PoseStamped()
            ps.header.frame_id = self.fixed_frame
            ps.header.stamp = stamp
            ps.pose.position.x, ps.pose.position.y = est.position
            yaw = math.atan2(est.velocity[1], est.velocity[0])
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.pose_pub.publish(ps)
            pt = PointStamped()
            pt.header = ps.header
            pt.point.x, pt.point.y = est.position
            self.point_pub.publish(pt)
        self._publish_markers(tracks, est, stamp)

    def _publish_markers(self, tracks, est, stamp):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, (tid, pos, vel, st) in enumerate(tracks):
            is_host = tid == est.track_id
            m = Marker()
            m.header.frame_id = self.fixed_frame
            m.header.stamp = stamp
            m.ns, m.id, m.type = "tracks", i, Marker.CYLINDER
            m.pose.position.x, m.pose.position.y, m.pose.position.z = pos[0], pos[1], 0.9
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.45
            m.scale.z = 1.8
            m.color = ColorRGBA(r=0.1, g=0.9, b=0.1, a=0.6) if is_host else (
                ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.4) if st == TrackState.CONFIRMED
                else ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.3))
            arr.markers.append(m)
            a = Marker()
            a.header = m.header
            a.ns, a.id, a.type = "velocity", i, Marker.ARROW
            a.points = [Point(x=pos[0], y=pos[1], z=1.9),
                        Point(x=pos[0] + vel[0], y=pos[1] + vel[1], z=1.9)]
            a.scale.x, a.scale.y, a.scale.z = 0.05, 0.1, 0.1
            a.color = m.color
            a.color.a = 1.0
            arr.markers.append(a)
        if est.status == HostStatus.LOST and est.position is not None:
            m = Marker()
            m.header.frame_id = self.fixed_frame
            m.header.stamp = stamp
            m.ns, m.id, m.type = "lost", 0, Marker.SPHERE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = est.position[0], est.position[1], 0.9
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.4
            m.color = ColorRGBA(r=1.0, g=0.2, b=0.1, a=0.8)
            arr.markers.append(m)
        self.marker_pub.publish(arr)

    def _diagnostics(self):
        now = time.monotonic()
        self.det_times = [t for t in self.det_times if now - t < 5.0]
        lat = self.latencies[-20:]
        self.latencies = lat
        est = getattr(self, "estimate", None)
        if not self.enabled:
            return
        self.get_logger().info(
            f"detections {len(self.det_times) / 5.0:.1f} Hz, latency "
            f"{(np.mean(lat) * 1000 if lat else float('nan')):.0f} ms, "
            f"tracks {len(self.tracker.tracks)}, host {est.status.value if est else '-'}, "
            f"frames {self.latest_seq}, camera_info {self.camera_info is not None}, "
            f"yolo ready {self.yolo.service_is_ready()}, {self.diag}"
        )
        self.diag = {k: 0 for k in self.diag}

    # ------------------------------------------------------------------ head

    def _head(self):
        est = getattr(self, "estimate", None)
        if not self.head_enabled or est is None or est.position is None:
            return
        if est.status == HostStatus.NO_HOST:
            return
        la = self.get_parameter("head_lookahead").value
        tx = est.position[0] + est.velocity[0] * la
        ty = est.position[1] + est.velocity[1] * la
        robot = self._robot_pose()
        if robot is None:
            return
        bearing = math.atan2(ty - robot[1], tx - robot[0])
        now = time.monotonic()
        if self.last_head_target is not None:
            db = abs(math.atan2(math.sin(bearing - self.last_head_target),
                                math.cos(bearing - self.last_head_target)))
            if db < math.radians(self.get_parameter("head_deadband_deg").value) and now - self.last_head_time < 2.0:
                return
        if not self.head_client.server_is_ready():
            return
        goal = PointHead.Goal()
        goal.target = PointStamped()
        goal.target.header.frame_id = self.fixed_frame
        # Up close only the torso is in view and YOLO misses a head-on-torso crop: aim
        # lower the closer the host is (1.1 m at >= 2 m down to 0.8 m at <= 1 m).
        rng = math.hypot(tx - robot[0], ty - robot[1])
        z_far = float(self.get_parameter("head_target_z").value)
        z = z_far - 0.3 * min(1.0, max(0.0, (2.0 - rng) / 1.0))
        goal.target.point = Point(x=tx, y=ty, z=z)
        goal.pointing_frame = "head_2_link"
        goal.pointing_axis.x = 1.0
        goal.max_velocity = 1.5
        goal.min_duration = Duration(seconds=0.1).to_msg()
        self.head_client.send_goal_async(goal)  # fire and forget: never block tracking
        self.last_head_target = bearing
        self.last_head_time = now

    # ------------------------------------------------------------------ services

    def _lock_srv(self, req, res):
        robot = self._robot_pose()
        if robot is None:
            res.success, res.message = False, "no robot pose"
            return res
        with self.lock:
            ok = self.host.try_lock(robot)
            res.success = ok
            res.message = f"locked track {self.host.host_id}" if ok else "nobody in front"
        self.get_logger().info(f"lock_host: {res.message}")
        return res

    def _release_srv(self, req, res):
        with self.lock:
            self.host.reset()
        res.success, res.message = True, "released"
        return res

    def _new_leg_srv(self, req, res):
        with self.lock:
            self.host.reset_travel()
        res.success, res.message = True, "new leg"
        return res

    def _enable_srv(self, req, res):
        with self.lock:
            if not req.data:
                self.host.reset()
                self.tracker.tracks = []
            self.enabled = req.data
        self.last_head_target = None
        res.success, res.message = True, f"tracker {'enabled' if req.data else 'disabled'}"
        self.get_logger().info(res.message)
        return res

    def _head_srv(self, req, res):
        self.head_enabled = req.data
        self.last_head_target = None
        res.success, res.message = True, f"head tracking {'on' if req.data else 'off'}"
        return res


def main():
    rclpy.init()
    node = PersonTrackerNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
