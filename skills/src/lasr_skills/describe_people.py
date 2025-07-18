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

# from numpy2message import numpy2message
from .vision import GetImage
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
                GetImage(),
                transitions={
                    "succeeded": "GET_CLIP_ATTRIBUTES",
                    "failed": "failed",
                },
                remapping={"img_msg": "img_raw"},
            )

            smach.StateMachine.add(
                "GET_CLIP_ATTRIBUTES",
                self.GetClipAttributes(),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )

    class GetClipAttributes(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["img_raw"],
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
                    image_raw=userdata.img_raw,
                )
                glasses_response = self.clip_service(glasses_request)
                hat_request = VqaRequest(
                    possible_answers=self.hat_questions, image_raw=userdata.img_raw
                )
                hat_response = self.clip_service(hat_request)
                hair_request = VqaRequest(
                    possible_answers=self.hair_questions, image_raw=userdata.img_raw
                )
                hair_response = self.clip_service(hair_request)
                t_shirt_request = VqaRequest(
                    possible_answers=self.t_shirt_questions,
                    image_raw=userdata.img_raw,
                )
                t_shirt_response = self.clip_service(t_shirt_request)
                rospy.loginfo("RESPONSES")
                rospy.loginfo(f"Glasses: {glasses_response}")
                rospy.loginfo(f"Hat: {hat_response}")
                rospy.loginfo(f"Hair: {hair_response}")
                rospy.loginfo(f"T-shirt: {t_shirt_response}")

                glasses_bool = glasses_response.answer == "a person wearing glasses"
                hat_bool = hat_response.answer == "a person wearing a hat"
                hair_bool = hair_response.answer == "a person with long hair"
                t_shirt_bool = (
                    t_shirt_response.answer == "a person wearing a short-sleeve t-shirt"
                )

                rospy.loginfo("DETECTED ATTRIBUTES")
                rospy.loginfo(f"Glasses: {glasses_bool}")
                rospy.loginfo(f"Hat: {hat_bool}")
                rospy.loginfo(f"Hair: {hair_bool}")
                rospy.loginfo(f"Short Sleeve T-shirt: {t_shirt_bool}")

                clip_detection_dict = {
                    "glasses": glasses_bool,
                    "hat": hat_bool,
                    "long_hair": hair_bool,
                    "short_sleeve_t_shirt": t_shirt_bool,
                }

                rospy.loginfo(clip_detection_dict)
                userdata.clip_detection_dict = clip_detection_dict
            except Exception as e:
                rospy.logerr(f"Failed to get clip attributes: {e}")
                return "failed"
            return "succeeded"
