"""The receptionist version of learn faces uses userdata for the name of the guest instead"""

import rospy
import smach
<<<<<<< HEAD

from lasr_vision_msgs.srv import AddFace
from lasr_skills.vision import CropImage3D
from lasr_skills import Detect3D
from cv_bridge import CvBridge


class ReceptionistLearnFaces(smach.StateMachine):

    class LearnFaceState(smach.State):
        def __init__(self, guest_id: str):
            self._guest_id = guest_id
            self._bridge = CvBridge()
            self._learn_face = rospy.ServiceProxy("/lasr_vision_reid/add_face", AddFace)
            self._learn_face.wait_for_service()
            smach.State.__init__(
                self, outcomes=["succeeded", "failed"], input_keys=["cropped_images"]
            )

        def execute(self, userdata):
            self._learn_face()
            try:
                success = self._learn_face(
                    self._bridge.cv2_to_imgmsg(
                        userdata.cropped_images["person"], encoding="rgb8"
                    ),
                    self._guest_id,
                )
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
                return "failed"

            return "succeeded" if success else "failed"

    def __init__(self, guest_id: str, dataset_size: int = 10):
=======

from lasr_vision_msgs.srv import (
    AddFace,
    AddFaceRequest,
    YoloPoseDetection,
    YoloPoseDetectionRequest,
)
from lasr_skills.vision import CropImage3D
from lasr_skills import Detect3D
from cv_bridge import CvBridge


class ReceptionistLearnFaces(smach.StateMachine):

    class CheckEyes(smach.State):
        """Checks if eyes are present in a given RGB image"""

        _yolo_service: rospy.ServiceProxy

        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["image_raw"],
            )

            self._yolo_service = rospy.ServiceProxy(
                "/yolo/detect_pose",
                YoloPoseDetection,
            )
            self._yolo_service.wait_for_service()

        def execute(self, userdata):
            image_raw = userdata.image_raw
            req = YoloPoseDetectionRequest()
            req.image_raw = image_raw
            req.model = "yolo11n-pose.pt"
            req.confidence = 0.5

            try:
                response = self._yolo_service(req)
                if not response.detections:
                    result = "failed"
                else:
                    for keypoint_detection in response.detections:
                        for keypoint in keypoint_detection.keypoints:
                            if "eye" in keypoint.keypoint_name.lower():
                                result = "succeeded"
                                break
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
                return "failed"

            return result

    class LearnFaceState(smach.State):
        def __init__(self, guest_id: str):
            self._guest_id = guest_id
            self._bridge = CvBridge()
            self._learn_face = rospy.ServiceProxy("/lasr_vision_reid/add_face", AddFace)
            self._learn_face.wait_for_service()
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["cropped_images", "num_images"],
                output_keys=["num_images"],
            )

        def execute(self, userdata):
            try:
                request = AddFaceRequest(
                    image_raw=self._bridge.cv2_to_imgmsg(
                        userdata.cropped_images["person"], encoding="rgb8"
                    ),
                    name=self._guest_id,
                )
                response = self._learn_face(request)
                if response.success:
                    userdata.num_images += 1
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
                return "failed"

            return "succeeded"

    class CheckDoneState(smach.State):
        def __init__(self, dataset_size: int):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["num_images"],
            )
            self._dataset_size = dataset_size

        def execute(self, userdata):
            if userdata.num_images >= self._dataset_size:
                rospy.loginfo("Collected enough images for the guest.")
                return "succeeded"
            else:
                rospy.logwarn(
                    f"Not enough images collected for the guest: {userdata.num_images}/{self._dataset_size}."
                )
                return "failed"

    def __init__(self, guest_id: str, dataset_size: int = 5):
>>>>>>> origin/main
        smach.StateMachine.__init__(
            self, outcomes=["succeeded", "failed"], input_keys=["guest_data"]
        )
        self._guest_id = guest_id
        self._dataset_size = dataset_size

<<<<<<< HEAD
        with self:
            self.userdata.guest_images = []
=======
        # TODO:
        # Should add a check for detecting eyes in image befor learning face.
        with self:
            self.userdata.num_images = 0
>>>>>>> origin/main
            smach.StateMachine.add(
                "DETECT_3D",
                Detect3D(
                    filter=["person"],
                ),
                transitions={
<<<<<<< HEAD
                    "succeeded": "CROP_IMAGE_3D",
=======
                    "succeeded": "CHECK_EYES",
>>>>>>> origin/main
                    "failed": "failed",
                },
            )
            smach.StateMachine.add(
<<<<<<< HEAD
=======
                "CHECK_EYES",
                self.CheckEyes(),
                transitions={
                    "succeeded": "CROP_IMAGE_3D",
                    "failed": "DETECT_3D",
                },
            )
            smach.StateMachine.add(
>>>>>>> origin/main
                "CROP_IMAGE_3D",
                CropImage3D(
                    filters=["person"],
                    crop_logic="nearest",
                    crop_type="masked",
                ),
                transitions={
<<<<<<< HEAD
                    "succeeded": "APPEND_DETECTIONS",
                    "failed": "failed",
                },
            )
            smach.StateMachine.add(
                "APPEND_DETECTIONS",
                self.AppendDetections(number_of_images=self._dataset_size),
                transitions={
                    "succeeded": "DETECT_3D",
                    "finished": "LEARN_FACE",
                    "failed": "failed",
=======
                    "succeeded": "LEARN_FACE",
                    "failed": "failed",
                },
                remapping={
                    "cropped_images": "cropped_images",
>>>>>>> origin/main
                },
            )

            smach.StateMachine.add(
                "LEARN_FACE",
                self.LearnFaceState(self._guest_id),
                transitions={
<<<<<<< HEAD
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )
=======
                    "succeeded": "CHECK_DONE",
                    "failed": "failed",
                },
            )
            smach.StateMachine.add(
                "CHECK_DONE",
                self.CheckDoneState(self._dataset_size),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "DETECT_3D",
                },
            )
>>>>>>> origin/main
