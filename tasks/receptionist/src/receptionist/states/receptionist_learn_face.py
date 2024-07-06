"""The receptionist version of learn faces uses userdata for the name of the guest instead"""

import smach
import rospy
import sys


from lasr_vision_msgs.msg import CDRequest, CDResponse
from lasr_vision_msgs.srv import (
    LearnFace,
    LearnFaceRequest,
    CroppedDetection,
    CroppedDetectionRequest,
    CroppedDetectionResponse,
)

from sensor_msgs.msg import Image

from typing import List


class ReceptionistLearnFaces(smach.State):

    _guest_id: str
    _dataset_size: int
    _learn_face: rospy.ServiceProxy

    def __init__(self, guest_id: str, dataset_size: int = 10):
        smach.State.__init__(
            self, outcomes=["succeeded", "failed"], input_keys=["guest_data"]
        )
        self._guest_id = guest_id
        self._dataset_size = dataset_size

        rospy.wait_for_service("/learn_face")
        self._learn_face = rospy.ServiceProxy("/learn_face", LearnFace)

        rospy.wait_for_service("/vision/cropped_detection")
        self._cropped_detection = rospy.ServiceProxy(
            "/vision/cropped_detection", CroppedDetection
        )

    def execute(self, userdata) -> str:

        cropped_detection_req: CroppedDetectionRequest = CroppedDetectionRequest(
            [
                CDRequest(
                    method="closest",
                    use_mask=True,
                    yolo_model="yolov8x-seg.pt",
                    yolo_model_confidence=0.5,
                    yolo_nms_threshold=0.3,
                    object_names=["person"],
                )
            ]
            * self._dataset_size
        )

        try:

            cropped_detection_resp: CroppedDetectionResponse = self._cropped_detection(
                cropped_detection_req
            )
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return "failed"

        images: List[Image] = [
            resp.cropped_imgs[0]
            for resp in cropped_detection_resp.responses
            if resp.cropped_imgs
        ]

        learn_face_req: LearnFaceRequest = LearnFaceRequest(
            name=self._guest_id,
            dataset="receptionist",
            images=images,
        )

        try:
            self._learn_face(learn_face_req)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return "failed"

        return "succeeded"
