#!/usr/bin/env python3
from typing import Optional
import smach
import rospy
from lasr_skills.vision import GetImage
from lasr_vision_msgs.srv import BodyPixDetection, BodyPixDetectionRequest
from lasr_vision_msgs.msg import BodyPixMaskRequest


class DetectGesture(smach.State):
    """
    State for detecting gestures.
    """

    def __init__(
        self,
        gesture_to_detect: Optional[str] = None,
        buffer_width: int = 50,
    ):
        """Optionally stores the gesture to detect. If None, it will infer the gesture from the keypoints."""
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["img_msg"],
            output_keys=["gesture_detected"],
        )

        self.gesture_to_detect = gesture_to_detect
        self.body_pix_client = rospy.ServiceProxy("/bodypix/detect", BodyPixDetection)
        self.buffer_width = buffer_width

    def execute(self, userdata):

        body_pix_masks = BodyPixMaskRequest()
        body_pix_masks.parts = [
            "left_shoulder",
            "right_shoulder",
            "left_wrist",
            "right_wrist",
        ]
        masks = [body_pix_masks]

        req = BodyPixDetectionRequest()
        req.image_raw = userdata.img_msg
        req.masks = masks
        req.dataset = "resnet50"
        req.confidence = 0.7

        try:
            res = self.body_pix_client(req)
        except Exception as e:
            print(e)
            return "failed"

        part_info = {}
        poses = res.poses
        for pose in poses:
            for keypoint in pose.keypoints:
                part_info[keypoint.part] = {
                    "x": keypoint.xy[0],
                    "y": keypoint.xy[1],
                    "score": keypoint.score,
                }
        print(f"Gesture to detect: {self.gesture_to_detect}")
        if (
            self.gesture_to_detect == "person raising their left arm"
            or self.gesture_to_detect is None
        ):
            if part_info["leftWrist"]["y"] < part_info["leftShoulder"]["y"]:
                self.gesture_to_detect = "person raising their left arm"
        if (
            self.gesture_to_detect == "person raising their right arm"
            or self.gesture_to_detect is None
        ):
            if part_info["rightWrist"]["y"] < part_info["rightShoulder"]["y"]:
                self.gesture_to_detect = "person raising their right arm"
        if (
            self.gesture_to_detect == "person pointing to the left"
            or self.gesture_to_detect is None
        ):
            if (
                part_info["leftWrist"]["x"] - self.buffer_width
                > part_info["leftShoulder"]["x"]
            ):
                self.gesture_to_detect = "person pointing to the left"
        if (
            self.gesture_to_detect == "person pointing to the right"
            or self.gesture_to_detect is None
        ):
            if (
                part_info["rightShoulder"]["x"] - self.buffer_width
                > part_info["rightWrist"]["x"]
            ):
                self.gesture_to_detect = "person pointing to the right"
        if (
            self.gesture_to_detect == "person pointing forwards"
            or self.gesture_to_detect is None
        ):
            if (
                part_info["leftWrist"]["y"] > part_info["leftShoulder"]["y"]
                and part_info["rightWrist"]["y"] > part_info["rightShoulder"]["y"]
                and (
                    abs(part_info["leftWrist"]["x"] - part_info["leftShoulder"]["x"])
                    > self.buffer_width
                    or abs(
                        part_info["rightWrist"]["x"] - part_info["rightShoulder"]["x"]
                    )
                    > self.buffer_width
                )
            ):
                self.gesture_to_detect = "person pointing forwards"

        if self.gesture_to_detect is None:
            self.gesture_to_detect = "none"

        userdata.gesture_detected = self.gesture_to_detect

        return "succeeded"


### For example usage:
class GestureDetectionSM(smach.StateMachine):
    """
    State machine for detecting gestures.
    """

    def __init__(self, gesture_to_detect: Optional[str] = None):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])
        self.gesture_to_detect = gesture_to_detect
        with self:
            smach.StateMachine.add(
                "GET_IMAGE",
                GetImage(),
                transitions={"succeeded": "BODY_PIX_DETECTION", "failed": "failed"},
            )

            smach.StateMachine.add(
                "BODY_PIX_DETECTION",
                DetectGesture(gesture_to_detect=self.gesture_to_detect),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )


if __name__ == "__main__":
    import random

    rospy.init_node("gesture_detection_sm")
    gestures_to_detect = [
        # "person raising their left arm",
        # "person raising their right arm",
        "person pointing to the left",
        "person pointing to the right",
        "person pointing forwards",
    ]
    ### Example usage:
    while not rospy.is_shutdown():
        gesture = random.choice(gestures_to_detect)
        sm = GestureDetectionSM()
        sm.execute()
        print("Gesture detected:", sm.userdata.gesture_detected)

    rospy.spin()
