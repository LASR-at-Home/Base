"""The receptionist version of learn faces uses userdata for the name of the guest instead"""

import smach
import rospy
import sys

from lasr_vision_msgs.srv import (
    LearnFace,
    LearnFaceRequest,
)
from sensor_msgs.msg import Image
from lasr_skills.vision import CropImage3D
from lasr_skills import Detect3D
from cv2_img import msg_to_cv2_img, cv2_img_to_msg

from typing import List


class ReceptionistLearnFaces(smach.StateMachine):
    class AppendDetections(smach.State):
        def __init__(self, number_of_images: int = 10):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed", "finished"],
                input_keys=["cropped_images", "guest_images"],
                output_keys=["guest_images"],
            )
            self._number_of_images = number_of_images

        def execute(self, userdata) -> str:
            userdata.guest_images.append(userdata.cropped_images["person"])
            if len(userdata.guest_images) >= self._number_of_images:
                return "finished"
            return "succeeded"

    class LearnFaceState(smach.State):
        def __init__(self, guest_id: str):
            self._guest_id = guest_id
            self._learn_face = rospy.ServiceProxy("/learn_face", LearnFace)
            self._learn_face.wait_for_service()
            smach.State.__init__(
                self, outcomes=["succeeded", "failed"], input_keys=["guest_images"]
            )

        def execute(self, userdata):
            images = userdata.guest_images
            image_messages = [cv2_img_to_msg(img) for img in images]
            learn_face_req = LearnFaceRequest(
                name=self._guest_id,
                dataset="receptionist",
                images=image_messages,
            )
            try:
                self._learn_face(learn_face_req)
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
                return "failed"

            return "succeeded"

    def __init__(self, guest_id: str, dataset_size: int = 10):
        smach.StateMachine.__init__(
            self, outcomes=["succeeded", "failed"], input_keys=["guest_data"]
        )
        self._guest_id = guest_id
        self._dataset_size = dataset_size

        self._learn_face = rospy.ServiceProxy("/learn_face", LearnFace)
        rospy.wait_for_service("/learn_face")

        with self:
            self.userdata.guest_images = []
            smach.StateMachine.add(
                "DETECT_3D",
                Detect3D(
                    filter=["person"],
                ),
                transitions={
                    "succeeded": "CROP_IMAGE_3D",
                    "failed": "failed",
                },
            )
            smach.StateMachine.add(
                "CROP_IMAGE_3D",
                CropImage3D(
                    filters=["person"],
                    crop_logic="nearest",
                    crop_type="masked",
                ),
                transitions={
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
                },
            )

            smach.StateMachine.add(
                "LEARN_FACE",
                self.LearnFaceState(self._guest_id),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )
