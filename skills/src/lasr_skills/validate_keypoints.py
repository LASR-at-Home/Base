"""This skill checks whether a set of required bodypix keypoints can be detected in a given image."""

#!/usr/bin/env python3
from typing import Optional
import smach
import rospy
import cv2
import cv2_img
from lasr_vision_msgs.srv import BodyPixDetection, BodyPixDetectionRequest
from lasr_vision_msgs.msg import BodyPixMaskRequest
from sensor_msgs.msg import Image


class ValidateKeypoints(smach.State):

    def __init__(
        self,
        keypoints_to_detect: list[str],
        bodypix_model: str = "resnet50",
        bodypix_confidence: float = 0.7,
    ):
        """Takes a list of keypoints to check for in the image. If any are missing, this will be returned
        in the userdata.

        Args:
            keypoints_to_detect (list[str]): List of keypoints to check for in the image.

            bodypix_model (str, optional): The bodypix model to use. Defaults to "resnet50".

            bodypix_confidence (float, optional): The confidence threshold for bodypix. Defaults to 0.7.


        """
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["img_msg"],
            output_keys=["missing_keypoints"],
        )
        self._keypoints_to_detect = keypoints_to_detect
        self._bodypix_model = bodypix_model
        self._bodypix_confidence = bodypix_confidence
        self._bodypix_client = rospy.ServiceProxy("/bodypix/detect", BodyPixDetection)

    def execute(self, userdata):

        body_pix_masks = BodyPixMaskRequest()
        body_pix_masks.parts = self.keypoints_to_detect
        masks = [body_pix_masks]

        req = BodyPixDetectionRequest()
        req.image_raw = userdata.img_msg
        req.masks = masks
        req.dataset = self._bodypix_model
        req.confidence = self._bodypix_confidence

        try:
            res = self._bodypix_client(req)
        except Exception as e:
            print(e)
            return "failed"

        part_info = {}
        for mask in res.masks:
            part_info[mask.part] = mask.mask

        missing_keypoints = [
            keypoint
            for keypoint in self.keypoints_to_detect
            if keypoint not in part_info
        ]

        if missing_keypoints:
            rospy.logwarn(f"Missing keypoints: {missing_keypoints}")
            userdata.missing_keypoints = missing_keypoints
            return "failed"
        return "succeeded"
