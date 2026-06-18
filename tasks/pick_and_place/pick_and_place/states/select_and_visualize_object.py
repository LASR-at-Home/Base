import cv2
import yasmin
import yasmin_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, DurabilityPolicy


class SelectAndVisualiseObject(yasmin.State):
    """
    Selects an object from the detected_objects list and publishes a
    debug image with a bounding box to /referee_view so the referee can
    confirm the robot's selection.

    Without a target_name, selects the first detected object — used by
    the table/extra-surface cleanup loops where order doesn't matter.
    With a target_name, selects the specific named object from the list
    — used by breakfast setup, where DetectObjects(queries=["bowl","spoon"])
    can return either order and a specific one needs to be picked out.
    Reuses the cached image set on the blackboard by DetectObjects
    ("last_rgb_image") rather than re-fetching a fresh camera frame,
    so the visualisation matches exactly what was detected.

    Constructor args:
        target_name : str | None — object name to search for; None

                                    selects the first detection
    Blackboard inputs:
        detected_objects : List[Detection3D]
        last_rgb_image    : Image — set by DetectObjects

    Blackboard outputs:
        selected_object      : Detection3D
        selected_object_name : str
    """

    def __init__(self, target_name: str = None):
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
        if not detected:
            yasmin.YASMIN_LOG_WARN("No detected objects to select from.")
            return "failed"

        if self._target_name is not None:
            selected = next(
                (obj for obj in detected if obj.name == self._target_name), None
            )
            if selected is None:
                yasmin.YASMIN_LOG_WARN(
                    f"'{self._target_name}' not found in detected_objects."
                )
                return "failed"
        else:
            # Default behaviour for cleanup loops — always take the first
            selected = detected[0]

        blackboard["selected_object"] = selected
        blackboard["selected_object_name"] = selected.name
        yasmin.YASMIN_LOG_INFO(f"Selected object: {selected.name}")
        self._publish_visualisation(selected, blackboard)
        return "succeeded"

    def _publish_visualisation(self, detection, blackboard) -> None:
        """
        Draws a bounding box and label on the cached detection-time image
        and publishes it to /referee_view, satisfying rule 16's perception
        communication requirement.
        """
        try:
            image_msg = blackboard.get("last_rgb_image")
            if image_msg is None:
                yasmin.YASMIN_LOG_WARN("No cached image available for visualisation.")
                return
            cv_im = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding="rgb8")
            xywh = detection.xywh  # top-left format from DetectObjects
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
