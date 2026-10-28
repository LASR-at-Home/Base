import rclpy
import yasmin
import tf2_ros
from yasmin_ros import ServiceState
from time import sleep
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from lasr_vision_interfaces.srv import YoloPoseDetection3D
from visualization_msgs.msg import Marker


class DetectWave(ServiceState):
    def __init__(
        self,
        image_topic="/head_front_camera/rgb/image_raw",
        depth_topic="/head_front_camera/depth/image_raw",
        camera_info_topic="/head_front_camera/depth/camera_info",
        model="yolo11n-pose.pt",
        confidence=0.5,
        target_frame="map",
        max_arm=0.9,
        raise_margin=0.15,
        elbow_tolerance=0.15,
    ):
        super().__init__(
            srv_type=YoloPoseDetection3D,
            srv_name="/yolo/detect3d_pose",
            create_request_handler=self._create_req,
            outcomes=["waving", "not_waving", "failed"],
            response_handler=self._response_handler,
        )
        self.set_description("Detect a waving customer via YOLO 3D pose")
        self.add_output_key("wave_detected")
        self.add_output_key("wave_position")
        self.model = model
        self.confidence = confidence
        self.target_frame = target_frame
        self._marker_pub = self._node.create_publisher(Marker, "/wave_debug_marker", 1)
        self.max_arm = max_arm
        self.raise_margin = raise_margin
        self.elbow_tolerance = elbow_tolerance
        self._buf = tf2_ros.Buffer()
        self._tf = tf2_ros.TransformListener(self._buf, self._node)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        img_sub = message_filters.Subscriber(
            self._node, Image, image_topic, qos_profile=qos
        )
        depth_sub = message_filters.Subscriber(
            self._node, Image, depth_topic, qos_profile=qos
        )
        info_sub = message_filters.Subscriber(
            self._node, CameraInfo, camera_info_topic, qos_profile=qos
        )
        self._info_cache = message_filters.Cache(info_sub, 10)
        self._ts = message_filters.ApproximateTimeSynchronizer(
            [img_sub, depth_sub], queue_size=10, slop=1.0
        )
        self._data = None

    def _create_req(self, blackboard):
        self._data = None

        def cb(img, depth):
            if self._data is not None:
                return
            info = self._info_cache.getLast()
            if info is None:
                return
            self._data = (img, depth, info)

        self._ts.registerCallback(cb)
        while self._data is None:
            yasmin.YASMIN_LOG_INFO("waiting for camera")
            sleep(0.5)

        img, depth, info = self._data
        req = YoloPoseDetection3D.Request()
        req.image_raw = img
        req.depth_image = depth
        req.depth_camera_info = info
        req.model = self.model
        req.confidence = self.confidence
        req.target_frame = self.target_frame
        return req

    def _raised_arm(self, kp, side):
        shoulder = kp.get(f"{side}_shoulder")
        elbow = kp.get(f"{side}_elbow")
        wrist = kp.get(f"{side}_wrist")
        if shoulder is None or elbow is None or wrist is None:
            return None
        # hand clearly above the shoulder, and the elbow up too, so one noisy wrist point is not enough
        if wrist.z < shoulder.z + self.raise_margin:
            return None
        if elbow.z < shoulder.z - self.elbow_tolerance:
            return None
        return shoulder, wrist

    def _arm_heights(self, kp):
        parts = []
        for side in ("left", "right"):
            sh = kp.get(f"{side}_shoulder")
            if sh is None:
                parts.append(f"{side}: no shoulder")
                continue
            el, wr = kp.get(f"{side}_elbow"), kp.get(f"{side}_wrist")
            el_s = f"{el.z - sh.z:+.2f}" if el is not None else "none"
            wr_s = f"{wr.z - sh.z:+.2f}" if wr is not None else "none"
            parts.append(f"{side} elbow {el_s} wrist {wr_s}")
        return ", ".join(parts)

    def _robot_xy(self):
        try:
            t = self._buf.lookup_transform(self.target_frame, "base_footprint", Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"no robot pose, ranking by distance to origin: {e}")
            return 0.0, 0.0

    def _response_handler(self, blackboard, response):
        best_point, best_dist = None, None
        rx, ry = self._robot_xy()
        for i, det in enumerate(response.detections):
            kp = {k.keypoint_name: k.point for k in det.keypoints}
            arm = self._raised_arm(kp, "left") or self._raised_arm(kp, "right")
            if arm is None:
                yasmin.YASMIN_LOG_INFO(f"person {i}: not waving ({self._arm_heights(kp)})")
                continue
            shoulder, wrist = arm

            length = (
                (wrist.x - shoulder.x) ** 2
                + (wrist.y - shoulder.y) ** 2
                + (wrist.z - shoulder.z) ** 2
            ) ** 0.5
            if length > self.max_arm:
                yasmin.YASMIN_LOG_WARN(f"person {i}: rejected, arm={length:.2f}m (mis-grouped?)")
                continue

            d = (shoulder.x - rx) ** 2 + (shoulder.y - ry) ** 2
            yasmin.YASMIN_LOG_INFO(
                f"person {i}: waving at ({shoulder.x:.2f}, {shoulder.y:.2f}), {d ** 0.5:.2f}m from robot"
            )
            if best_dist is None or d < best_dist:
                best_dist, best_point = d, shoulder

        if best_point is None:
            blackboard["wave_detected"] = False
            blackboard["wave_position"] = PointStamped()
            return "not_waving"

        yasmin.YASMIN_LOG_INFO(
            f"Waving customer at ({best_point.x:.2f}, {best_point.y:.2f})"
        )
        blackboard["wave_detected"] = True
        blackboard["wave_position"] = PointStamped(
            header=Header(frame_id=self.target_frame), point=best_point
        )
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = Time().to_msg()
        marker.ns, marker.id = "wave", 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = best_point
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3
        marker.color.r, marker.color.a = 1.0, 1.0
        self._marker_pub.publish(marker)
        return "waving"


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node(
        node_name="detect_wave",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )
    sm = yasmin.StateMachine(outcomes=["waving", "not_waving", "failed"])
    sm.add_state(
        "DETECT",
        DetectWave(),
        transitions={
            "waving": "waving",
            "not_waving": "not_waving",
            "failed": "failed",
        },
    )
    outcome = sm(yasmin.Blackboard())
    node.get_logger().info(f"DetectWave outcome: {outcome}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
