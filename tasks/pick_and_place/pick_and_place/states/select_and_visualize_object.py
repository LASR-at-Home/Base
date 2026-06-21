import cv2
import rclpy
import yasmin
import yasmin_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy


class SelectAndVisualiseObject(yasmin.State):
    """
    Pops the next object from the detected_objects list, announces it, and
    publishes a debug image with a bounding box to /referee_view so the referee
    can confirm the robot's selection.

    Announce-only: nothing is physically removed from the table, so the loop
    iterates the detected list (pop) instead of re-detecting each round. When the
    list is empty, every object has been processed → outcome "finished".

    Blackboard inputs:
        detected_objects : List[Detection3D]

    Blackboard outputs:
        selected_object      : Detection3D
        selected_object_name : str
        object_name          : str
    """

    def __init__(self, target_name: str = None):
        super().__init__(outcomes=["succeeded", "finished"])
        self.add_input_key("detected_objects")
        self.add_output_key("selected_object")
        self.add_output_key("selected_object_name")
        self.add_output_key("object_name")
        self._target_name = target_name
        self.node = yasmin_ros.logger_node
        self._bridge = CvBridge()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._referee_pub = self.node.create_publisher(Image, "/referee_view", qos)

        self._last_image = None
        cam_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.node.create_subscription(
            Image, "/head_front_camera/rgb/image_raw", self._on_image, cam_qos
        )

    def _on_image(self, msg):
        self._last_image = msg

    def execute(self, blackboard) -> str:
        detected = blackboard["detected_objects"]
        if not detected:
            # Announce-only mode: nothing is physically removed from the table,
            # so the loop iterates this list instead of re-detecting. Empty list
            # means every detected object has been processed → we are done.
            yasmin.YASMIN_LOG_INFO("No objects left to process — finished.")
            return "finished"

        if self._target_name is not None:
            selected = next(
                (obj for obj in detected if obj.name == self._target_name), None
            )
            if selected is None:
                yasmin.YASMIN_LOG_WARN(
                    f"'{self._target_name}' not found in detected_objects."
                )
                return "finished"
            detected.remove(selected)
        else:
            # Default behaviour for cleanup loops — always take the first
            selected = detected.pop(0)

        blackboard["detected_objects"] = detected
        blackboard["selected_object"] = selected
        blackboard["selected_object_name"] = selected.name
        blackboard["object_name"] = selected.name
        yasmin.YASMIN_LOG_INFO(
            f"Selected object: {selected.name} ({len(detected)} remaining)."
        )

        # ── 2. Announce to referee ───────────────────────────────────────────
        yasmin.YASMIN_LOG_INFO(
            "[TTS] I have selected an object, and it is displayed on my screen. "
            "Please take a look."
        )

        # ── 3. Publish visualisation ─────────────────────────────────────────
        self._publish_visualisation(selected)

        return "succeeded"

    def _publish_visualisation(self, detection) -> None:
        try:
            # # Grab the latest RGB image directly from the camera topic
            # success, image_msg = rclpy.wait_for_message.wait_for_message(
            #     msg_type=Image,
            #     node=self.node,
            #     topic="/head_front_camera/rgb/image_raw",
            #     time_to_wait=5.0,
            # )

            if self._last_image is None:
                yasmin.YASMIN_LOG_WARN("Could not get camera image for visualisation.")
                return

            label = detection.name
            xywh = detection.xywh
            confidence = detection.confidence

            cv_im = self._bridge.imgmsg_to_cv2(
                self._last_image, desired_encoding="rgb8"
            )

            cv2.rectangle(
                cv_im,
                (int(xywh[0]), int(xywh[1])),
                (int(xywh[0] + xywh[2]), int(xywh[1] + xywh[3])),
                (0, 255, 0),
                2,
            )
            cv2.putText(
                cv_im,
                f"{label} {confidence:.2f}",
                (int(xywh[0]), int(xywh[1] - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

            self._referee_pub.publish(
                self._bridge.cv2_to_imgmsg(cv_im, encoding="rgb8")
            )
            yasmin.YASMIN_LOG_INFO("Published visualisation to /referee_view.")

        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"Could not publish visualisation: {e}")
