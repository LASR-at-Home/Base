import rclpy
import yasmin
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

    def _response_handler(self, blackboard, response):
        best_point, best_dist = None, None
        for det in response.detections:
            kp = {k.keypoint_name: k.point for k in det.keypoints}
            shoulder = wrist = None
            yasmin.YASMIN_LOG_INFO(f"kp: {kp}")

            if (
                "left_wrist" in kp
                and "left_shoulder" in kp
                and kp["left_wrist"].z > kp["left_shoulder"].z
            ):
                shoulder, wrist = kp["left_shoulder"], kp["left_wrist"]
            elif (
                "right_wrist" in kp
                and "right_shoulder" in kp
                and kp["right_wrist"].z > kp["right_shoulder"].z
            ):
                shoulder, wrist = kp["right_shoulder"], kp["right_wrist"]
            if shoulder is None:
                continue

            arm = (
                (wrist.x - shoulder.x) ** 2
                + (wrist.y - shoulder.y) ** 2
                + (wrist.z - shoulder.z) ** 2
            ) ** 0.5
            if arm > self.max_arm:
                yasmin.YASMIN_LOG_WARN(f"rejected: arm={arm:.2f}m (mis-grouped?)")
                continue

            d = shoulder.x**2 + shoulder.y**2
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
