import math
import time

import yasmin
import yasmin_ros

import message_filters
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration as ROS2Duration
from rclpy.time import Time as ROS2Time

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header

from lasr_skills import LookToPoint
from lasr_vision_yolo.srv import YoloDetection3D


class DetectFloorTrash(yasmin.State):
    """
    Sweeps the head across a small ring of floor-level points around the
    trash bin's known position, since the trash item's exact location
    "near the trash bin" is not guaranteed by the rulebook.

    Stops sweeping as soon as one floor-level object is found, since the
    rulebook guarantees exactly one trash item on the floor.

    Uses YOLO (YoloDetection3D) instead of open-vocab detection since
    YOLO already returns 3D points directly — no manual depth projection
    or TF transform needed.

    Reads from ROS 2 params:
        pick_and_place.trash_bin.pose.position.x / .y
        pick_and_place.objects  (nested dict — extracts object names)

    Blackboard outputs:
        detected_objects : List[Detection3D]
    """

    FLOOR_Z_MAX        = 0.3   # anything below this height counts as floor-level
    SWEEP_RADIUS       = 0.6   # metres around the trash bin to look at
    SWEEP_POINTS_COUNT = 4     # how many points around the bin to check

    RGB_TOPIC   = "/head_front_camera/rgb/image_raw"
    DEPTH_TOPIC = "/head_front_camera/depth/image_raw"
    INFO_TOPIC  = "/head_front_camera/rgb/camera_info"

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_output_key("detected_objects")

        self.node = yasmin_ros.logger_node

        # ── Object query list from nested config dict ──────────────────────────
        try:
            objects_params = self.node.get_parameters_by_prefix("pick_and_place.objects")
            self._queries = list(set(
                key.split(".")[0] for key in objects_params.keys()
            ))
            if not self._queries:
                raise ValueError("Empty object list")
        except Exception:
            self._queries = ["object", "item", "trash"]

        # ── Compute sweep points around the trash bin ──────────────────────────
        try:
            bin_x = self.node.get_parameter(
                "pick_and_place.trash_bin.pose.position.x"
            ).value
            bin_y = self.node.get_parameter(
                "pick_and_place.trash_bin.pose.position.y"
            ).value
            self._sweep_points = self._compute_sweep_points(bin_x, bin_y)
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(
                f"Could not load trash_bin pose ({e}); using single forward look."
            )
            self._sweep_points = [Point(x=0.0, y=0.0, z=0.1)]

        # ── Synchronized camera capture ────────────────────────────────────────
        self._latest = None  # (rgb, depth, info) tuple, set by sync callback

        cam_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        img_sub   = message_filters.Subscriber(self.node, Image, self.RGB_TOPIC, qos_profile=cam_qos)
        depth_sub = message_filters.Subscriber(self.node, Image, self.DEPTH_TOPIC, qos_profile=cam_qos)
        info_sub  = message_filters.Subscriber(self.node, CameraInfo, self.INFO_TOPIC, qos_profile=cam_qos)

        self._info_cache = message_filters.Cache(info_sub, 10)
        self._ts = message_filters.ApproximateTimeSynchronizer(
            [img_sub, depth_sub], queue_size=10, slop=1.0
        )
        self._ts.registerCallback(self._sync_cb)

        # ── YOLO 3D detection client ───────────────────────────────────────────
        self._yolo = self.node.create_client(YoloDetection3D, "/yolo/detect3d")

    # ── Sweep point generation ─────────────────────────────────────────────────

    def _compute_sweep_points(self, bin_x: float, bin_y: float) -> list:
        """
        Generates floor-level points in a ring around the trash bin, so the
        robot looks in several directions to find the trash item regardless
        of exactly where near the bin it was placed.
        """
        points = []
        for i in range(self.SWEEP_POINTS_COUNT):
            angle = (2 * math.pi / self.SWEEP_POINTS_COUNT) * i
            x = bin_x + self.SWEEP_RADIUS * math.cos(angle)
            y = bin_y + self.SWEEP_RADIUS * math.sin(angle)
            points.append(Point(x=x, y=y, z=0.1))  # floor height
        return points

    # ── Camera sync callback ───────────────────────────────────────────────────

    def _sync_cb(self, img, depth):
        info = self._info_cache.getLast()
        if info is None:
            return
        self._latest = (img, depth, info)

    def _wait_for_synced_frame(self, timeout: float = 3.0):
        """Clears any stale frame and waits for a fresh synchronized capture."""
        self._latest = None
        start = time.time()
        while self._latest is None and time.time() - start < timeout:
            time.sleep(0.05)
        return self._latest

    # ── Detection at a single sweep point ─────────────────────────────────────

    @staticmethod
    def _wait_future(future, timeout=15.0):
        start = time.time()
        while not future.done():
            if time.time() - start > timeout:
                return None
            time.sleep(0.05)
        try:
            return future.result()
        except Exception:
            return None

    def _detect_at_current_point(self):
        """
        Captures a synchronized frame and runs YOLO 3D detection on it,
        returning detections filtered to floor-level objects only.
        YoloDetection3D already returns 3D points directly so no manual
        depth projection or TF transform is needed.
        """
        frame = self._wait_for_synced_frame(timeout=3.0)
        if frame is None:
            yasmin.YASMIN_LOG_WARN("No synchronized camera frame available.")
            return []

        rgb, depth, info = frame

        if not self._yolo.wait_for_service(timeout_sec=5.0):
            yasmin.YASMIN_LOG_WARN("YOLO service not available.")
            return []

        req = YoloDetection3D.Request()
        req.image_raw    = rgb
        req.depth_image  = depth
        req.camera_info  = info
        req.dataset      = "robocup.pt"  # TODO: update to your trained model name
        req.confidence   = 0.25
        req.nms          = 0.3

        resp = self._wait_future(self._yolo.call_async(req), timeout=15.0)
        if resp is None:
            return []

        # Filter to floor-level objects only via z-height
        floor_objects = [
            d for d in resp.detected_objects
            if d.point.z < self.FLOOR_Z_MAX
        ]
        return floor_objects

    # ── Main execution ─────────────────────────────────────────────────────────

    def execute(self, blackboard) -> str:
        yasmin.YASMIN_LOG_INFO(
            f"Sweeping {len(self._sweep_points)} floor points around the trash bin."
        )

        for i, point in enumerate(self._sweep_points):
            yasmin.YASMIN_LOG_INFO(
                f"Looking at sweep point {i+1}/{len(self._sweep_points)}: "
                f"({point.x:.2f}, {point.y:.2f}, {point.z:.2f})"
            )

            look = LookToPoint(
                pointstamped=PointStamped(
                    point=point,
                    header=Header(frame_id="map"),
                )
            )
            look.execute(blackboard)  # best-effort; continue regardless of outcome

            time.sleep(1.0)  # allow head/camera to settle

            floor_objects = self._detect_at_current_point()

            for obj in floor_objects:
                yasmin.YASMIN_LOG_INFO(
                    f"   Floor object: {obj.name} ({obj.confidence:.2f}) "
                    f"at ({obj.point.x:.2f}, {obj.point.y:.2f}, {obj.point.z:.2f})"
                )

            if floor_objects:
                yasmin.YASMIN_LOG_INFO(
                    f"Floor trash found: {floor_objects[0].name}. Stopping sweep."
                )
                blackboard["detected_objects"] = floor_objects[:1]
                return "succeeded"

        yasmin.YASMIN_LOG_INFO("No object found on the floor after full sweep.")
        return "failed"