import cv2
import yasmin
import yasmin_ros
from yasmin_ros.yasmin_node import YasminNode
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from pick_and_place import referee_view


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

    def __init__(self):
        super().__init__(outcomes=["succeeded", "finished"])
        self.add_input_key("detected_objects")
        self.add_output_key("selected_object")
        self.add_output_key("selected_object_name")
        self.add_output_key("object_name")
        self.add_input_key("handled_names")
        self.add_output_key("handled_names")
        self.node = yasmin_ros.logger_node
        self._bridge = CvBridge()

        # /referee_view is published by pick_and_place.referee_view (shared with DetectObjects)
        self._last_image = None
        cam_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
        self.node.create_subscription(
            Image, "/head_front_camera/rgb/image_raw", self._on_image, cam_qos)


    def _on_image(self, msg):
        self._last_image = msg




    def execute(self, blackboard) -> str:
        # ── 1. Select object ─────────────────────────────────────────────────
        detected = blackboard["detected_objects"]

        if not detected:
            # Announce-only mode: nothing is physically removed from the table,
            # so the loop iterates this list instead of re-detecting. Empty list
            # means every detected object has been processed → we are done.
            yasmin.YASMIN_LOG_INFO("No objects left to process — finished.")
            return "finished"

        # Pop the next object so the loop advances on each iteration.
        selected = detected.pop(0)
        blackboard["detected_objects"]     = detected
        blackboard["selected_object"]      = selected
        blackboard["selected_object_name"] = selected.name
        blackboard["object_name"] = selected.name
        try:   # DetectObjects will not offer it again this run
            blackboard["handled_names"] = list(blackboard["handled_names"] or []) + [selected.name]
        except Exception:
            blackboard["handled_names"] = [selected.name]
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
        # Redraw the frame DetectObjects ran on: every box, this one highlighted. No new
        # camera frame is grabbed, so the box always matches the picture.
        try:
            if referee_view.select(self.node, detection):
                yasmin.YASMIN_LOG_INFO("Published visualisation to /referee_view.")
            else:
                yasmin.YASMIN_LOG_WARN("No detection frame to visualise yet.")
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"Could not publish visualisation: {e}")
