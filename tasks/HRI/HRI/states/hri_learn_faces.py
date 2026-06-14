"""The HRI version of learn faces uses userdata for the name of the guest instead"""

import rclpy
from rclpy.node import Node

import yasmin
import yasmin_ros
from yasmin import State, StateMachine
from yasmin_ros import ServiceState

from lasr_vision_interfaces.srv import (
    AddFace,
    YoloPoseDetection,
)
from lasr_skills.vision import CropImage3D
from lasr_skills import Detect3D
from cv_bridge import CvBridge


class HRILearnFaces(StateMachine):
    class CheckEyes(ServiceState):
        """Checks if eyes are present in a given RGB image"""

        # _yolo_service: rospy.ServiceProxy

        def __init__(self):
            super().__init__(
                srv_type=YoloPoseDetection,
                srv_name="/yolo/detect_pose",
                create_request_handler=self._create_request,
                outcomes=["succeeded", "failed"],
                response_handler=self._handle_resp,
            )

            self.add_input_key("image_raw")

        def _create_request(self, blackboard):
            image_raw = blackboard["image_raw"]

            req = YoloPoseDetection.Request()
            req.image_raw = image_raw
            req.model = "yolo11n-pose.pt"
            req.confidence = 0.5
            # may need req.target_frame

            return req

        def _handle_resp(self, blackboard, response):
            try:
                if not response.detections:
                    return "failed"
                else:
                    for keypoint_detection in response.detections:
                        for keypoint in keypoint_detection.keypoints:
                            if "eye" in keypoint.keypoint_name.lower():
                                return "succeeded"

            except Exception as e:
                yasmin.YASMIN_LOG_ERROR(f"Service call failed: {e}")
                return "failed"

            return "failed"

    class LearnFaceState(ServiceState):
        def __init__(self, guest_id: str):
            super().__init__(
                srv_type=AddFace,
                srv_name="/lasr_vision_reid/add_face",
                create_request_handler=self._create_request,
                outcomes=["succeeded", "failed"],
                response_handler=self._handle_resp,
            )

            self.add_input_key("cropped_images")
            self.add_input_key("num_images")

            self.add_output_key("num_images")

            self._guest_id = guest_id
            self._bridge = CvBridge()

        def _create_request(self, blackboard):
            request = AddFace.Request()
            request.image_raw = self._bridge.cv2_to_imgmsg(
                blackboard["cropped_images"]["person"], encoding="rgb8"
            )
            request.name = self._guest_id
            return request

        def _handle_resp(self, blackboard, response):
            try:
                if response.success:
                    try:
                        blackboard["num_images"] += 1
                    except:
                        blackboard["num_images"] = 1
            except Exception as e:
                yasmin.YASMIN_LOG_ERROR(f"Service call failed: {e}")
                return "failed"

            return "succeeded"

    class CheckDoneState(State):
        def __init__(self, dataset_size: int):
            super().__init__(outcomes=["succeeded", "failed"])
            self.add_input_key("num_images")

            self._dataset_size = dataset_size

        def execute(self, blackboard):
            if blackboard['num_images'] >= self._dataset_size:
                yasmin.YASMIN_LOG_INFO("Collected enough images for the guest.")
                return "succeeded"
            else:
                num_images = blackboard["num_images"]
                yasmin.YASMIN_LOG_WARN(
                    f"Not enough images collected for the guest: {num_images}/{self._dataset_size}."
                )
                return "failed"

    def __init__(self, guest_id: str, dataset_size: int = 3):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)

        self.add_input_key("guest_data")

        self._guest_id = guest_id
        self._dataset_size = dataset_size

        self.add_state(
            "DETECT_3D",
            Detect3D(filter=["person"]),
            transitions={"succeeded": "CHECK_EYES", "failed": "failed"},
        )
        self.add_state(
            "CHECK_EYES",
            self.CheckEyes(),
            transitions={"succeeded": "CROP_IMAGE_3D", "failed": "DETECT_3D"},
        )
        self.add_state(
            "CROP_IMAGE_3D",
            CropImage3D(
                filters=["person"],
                crop_logic="nearest",
                crop_type="masked",
            ),
            transitions={"succeeded": "LEARN_FACE", "failed": "failed"},
        )

        self.add_state(
            "LEARN_FACE",
            self.LearnFaceState(self._guest_id),
            transitions={"succeeded": "CHECK_DONE", "failed": "failed"},
        )
        self.add_state(
            "CHECK_DONE",
            self.CheckDoneState(self._dataset_size),
            transitions={"succeeded": "succeeded", "failed": "DETECT_3D"},
        )
