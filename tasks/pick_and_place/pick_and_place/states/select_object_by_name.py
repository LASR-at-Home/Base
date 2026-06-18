import cv2
import yasmin
import yasmin_ros

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, DurabilityPolicy


class SelectObjectByName(yasmin.State):
    """
    Selects a specific named object from the detected_objects list and
    publishes a visualisation for the referee, same as
    SelectAndVisualiseObject but matching by name instead of always
    taking the first detection.

    Used for breakfast items, where a detection step returns more than
    one known item at once (e.g. bowl and spoon detected together) and
    a specific one needs to be picked out.

    Constructor args:
        target_name : str — the object name to search for, e.g. "bowl"

    Blackboard inputs:
        detected_objects : List[Detection3D]

    Blackboard outputs:
        selected_object      : Detection3D
        selected_object_name : str
    """

    def __init__(self, target_name: str):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("detected_objects")
        self.add_output_key("selected_object")
        self.add_output_key("selected_object_name")

        self._target_name = target_name
        self.node = yasmin_ros.logger_node
        self._bridge = CvBridge()

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._referee_pub = self.node.create_publisher(Image, "/referee_view", qos)

    def execute(self, blackboard) -> str:
        detected = blackboard["detected_objects"]

        selected = None
        for obj in detected:
            if obj.name == self._target_name:
                selected = obj
                break

        if selected is None:
            yasmin.YASMIN_LOG_WARN(
                f"'{self._target_name}' not found in detected_objects."
            )
            return "failed"

        blackboard["selected_object"]      = selected
        blackboard["selected_object_name"] = selected.name

        yasmin.YASMIN_LOG_INFO(f"Selected object: {selected.name}")

        self._publish_visualisation(selected)

        return "succeeded"

    def _publish_visualisation(self, detection) -> None:
        """
        Grabs the latest camera image and draws a bounding box for the
        referee. Same pattern as SelectAndVisualiseObject — no image is
        attached to Detection3D, so it must be fetched separately.
        """
        try:
            import rclpy
            success, image_msg = rclpy.wait_for_message.wait_for_message(
                msg_type=Image,
                node=self.node,
                topic="/head_front_camera/rgb/image_raw",
                time_to_wait=5.0,
            )
            if not success:
                yasmin.YASMIN_LOG_WARN("Could not get camera image for visualisation.")
                return

            cv_im = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding="rgb8")
            xywh  = detection.xywh

            cv2.rectangle(
                cv_im,
                (int(xywh[0]), int(xywh[1])),
                (int(xywh[0] + xywh[2]), int(xywh[1] + xywh[3])),
                (0, 255, 0),
                2,
            )
            cv2.putText(
                cv_im,
                f"{detection.name} {detection.confidence:.2f}",
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