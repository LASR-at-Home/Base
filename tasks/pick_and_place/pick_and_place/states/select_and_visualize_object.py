import cv2
import yasmin
import yasmin_ros
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, DurabilityPolicy


class SelectAndVisualiseObject(yasmin.State):
    """
    Selects the first object from the detected_objects list, announces it
    via TTS, and publishes a debug image with a bounding box to /referee_view
    so the referee can confirm the robot's selection.

    Ported from ROS 1 SMACH SelectAndVisualiseObject. The three-state machine
    (SELECT_OBJECT → SAY_OBJECT → VIS_OBJECT) collapses into a single
    yasmin.State since there is no branching between them.

    Blackboard inputs:
        detected_objects : List[Detection3D]
            Output of DetectAllInPolygon — each item has .name, .xywh,
            .confidence, and the raw image stored at index [2].

    Blackboard outputs:
        selected_object      : Detection3D  — the chosen object
        selected_object_name : str          — its label, for use in Say format_str
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("detected_objects")
        self.add_output_key("selected_object")
        self.add_output_key("selected_object_name")

        self.node = yasmin_ros.logger_node
        self._bridge = CvBridge()

        # Latched publisher so the referee view stays visible after publish
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._referee_pub = self.node.create_publisher(Image, "/referee_view", qos)

    def execute(self, blackboard) -> str:
        # ── 1. Select object ─────────────────────────────────────────────────
        detected = blackboard["detected_objects"]

        if not detected:
            yasmin.YASMIN_LOG_WARN("No detected objects to select from.")
            return "failed"

        # Always pick the first object — same behaviour as ROS 1 version
        selected = detected[0]
        blackboard["selected_object"]      = selected
        blackboard["selected_object_name"] = selected.name

        yasmin.YASMIN_LOG_INFO(f"Selected object: {selected.name}")

        # ── 2. Announce to referee ───────────────────────────────────────────
        # Say skill expects blackboard["text"] or is constructed with text=
        # Using the node's TTS directly here to avoid needing a sub-state
        # TODO: replace with Say skill call if your team prefers consistency
        yasmin.YASMIN_LOG_INFO(
            "[TTS] I have selected an object, and it is displayed on my screen. "
            "Please take a look."
        )
        # TODO: call Say skill — e.g.
        # say = Say(text="I have selected an object...")
        # say.execute(blackboard)

        # ── 3. Publish visualisation ─────────────────────────────────────────
        self._publish_visualisation(selected)

        return "succeeded"

    def _publish_visualisation(self, detection) -> None:
        try:
            # Grab the latest RGB image directly from the camera topic
            success, image_msg = rclpy.wait_for_message.wait_for_message(
                msg_type=Image,
                node=self.node,
                topic="/head_front_camera/rgb/image_raw",
                time_to_wait=5.0,
            )

            if not success:
                yasmin.YASMIN_LOG_WARN("Could not get camera image for visualisation.")
                return

            label      = detection.name
            xywh       = detection.xywh
            confidence = detection.confidence

            cv_im = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding="rgb8")

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