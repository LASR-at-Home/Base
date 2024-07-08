#!/usr/bin/env python3
"""This skill checks whether a set of required bodypix keypoints can be detected in a given image."""
from typing import List
import smach
import rospy
from lasr_vision_msgs.srv import (
    BodyPixKeypointDetection,
    BodyPixKeypointDetectionRequest,
)
from lasr_skills.vision import GetCroppedImage


class ValidateKeypoints(smach.State):

    def __init__(
        self,
        keypoints_to_detect: List[str],
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
        self._bodypix_client = rospy.ServiceProxy(
            "/bodypix/keypoint_detection", BodyPixKeypointDetection
        )

    def execute(self, userdata):
        req = BodyPixKeypointDetectionRequest()
        req.image_raw = userdata.img_msg
        req.dataset = self._bodypix_model
        req.confidence = self._bodypix_confidence

        try:
            res = self._bodypix_client(req)
        except Exception as e:
            print(e)
            return "failed"

        detected_keypoints = res.keypoints

        rospy.loginfo(f"Detected keypoints: {detected_keypoints}")

        keypoint_names = [keypoint.keypoint_name for keypoint in detected_keypoints]

        missing_keypoints = [
            keypoint
            for keypoint in self._keypoints_to_detect
            if keypoint not in keypoint_names
        ]

        if missing_keypoints:
            rospy.logwarn(f"Missing keypoints: {missing_keypoints}")
            userdata.missing_keypoints = missing_keypoints
            return "failed"
        return "succeeded"


if __name__ == "__main__":
    rospy.init_node("validate_keypoints")
    while not rospy.is_shutdown():
        get_cropped_image = GetCroppedImage(
            "person",
            crop_method="centered",
            rgb_topic="/usb_cam/image_raw",
        )
        sm = smach.StateMachine(outcomes=["succeeded", "failed"])
        with sm:
            smach.StateMachine.add(
                "GET_CROPPED_IMAGE",
                get_cropped_image,
                transitions={"succeeded": "VALIDATE_KEYPOINTS", "failed": "failed"},
            )
            smach.StateMachine.add(
                "VALIDATE_KEYPOINTS",
                ValidateKeypoints(["nose"]),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
        sm.execute()
        input("Press Enter to continue...")
