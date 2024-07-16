#!/usr/bin/env python3
import os
import cv2
import rospy
import smach
import cv2_img
import numpy as np

from lasr_vision_msgs.srv import (
    YoloDetection,
    BodyPixMaskDetection,
    BodyPixMaskDetectionRequest,
    TorchFaceFeatureDetectionDescription,
    Vqa,
    VqaRequest,
)
from numpy2message import numpy2message
from .vision import GetCroppedImage, ImageMsgToCv2
import numpy as np
from lasr_skills.validate_keypoints import ValidateKeypoints


class DescribePeople(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=[],
            output_keys=["clip_detection_dict"],
        )

        with self:
            smach.StateMachine.add(
                "GET_IMAGE",
                GetCroppedImage(
                    object_name="person",
                    method="closest",
                    use_mask=False,  # If true prediction can be very wrong!!!
                ),
                transitions={
                    "succeeded": "CLIP ATTRIBUTES",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                self.GetClipAttributes(),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )

    class GetClipAttributes(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["image_raw"],
                output_keys=["clip_detection_dict"],
            )
            self.clip_service = rospy.ServiceProxy("/clip_vqa/query_service", Vqa)
            self.glasses_questions = [
                "a person wearing glasses",
                "a person not wearing glasses",
            ]
            self.hat_questions = [
                "a person wearing a hat",
                "a person not wearing a hat",
            ]
            self.hair_questions = [
                "a person with long hair",
                "a person with short hair",
            ]
            self.t_shirt_questions = [
                "a person wearing a short-sleeve t-shirt",
                "a person wearing a long-sleeve t-shirt",
            ]

        def execute(self, userdata):
            try:
                glasses_request = VqaRequest(
                    possible_answers=self.glasses_questions,
                    image_raw=userdata.image_raw,
                )
                glasses_response = self.clip_service(glasses_request)
                hat_request = VqaRequest(
                    possible_answers=self.hat_questions, image_raw=userdata.image_raw
                )
                hat_response = self.clip_service(hat_request)
                hair_request = VqaRequest(
                    possible_answers=self.hair_questions, image_raw=userdata.image_raw
                )
                hair_response = self.clip_service(hair_request)
                t_shirt_request = VqaRequest(
                    possible_answers=self.t_shirt_questions,
                    image_raw=userdata.image_raw,
                )
                t_shirt_response = self.clip_service(t_shirt_request)

                glasses_bool = glasses_response.answer == "a person wearing glasses"
                hat_bool = hat_response.answer == "a person wearing a hat"
                hair_bool = hair_response.answer == "a person with long hair"
                t_shirt_bool = (
                    t_shirt_response == "a person wearing a short-sleeve t-shirt"
                )

                userdata.clip_detection_dict = {
                    "glasses": glasses_bool,
                    "hat": hat_bool,
                    "long_hair": hair_bool,
                    "short_sleeve_t_shirt": t_shirt_bool,
                }
            except Exception as e:
                rospy.logerr(f"Failed to get clip attributes: {e}")
                return "failed"
            return "succeeded"
