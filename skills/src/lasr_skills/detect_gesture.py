#!/usr/bin/env python3
from typing import Optional
import smach
import rospy
import cv2
import cv2_img
from lasr_skills.vision import GetImage
from lasr_skills import PlayMotion
from lasr_vision_msgs.srv import BodyPixDetection, BodyPixDetectionRequest
from lasr_vision_msgs.msg import BodyPixMaskRequest
from sensor_msgs.msg import Image


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
        self.debug_publisher = rospy.Publisher(
            "/gesture_detection/debug", Image, queue_size=1
        )
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
        if (
            self.gesture_to_detect == "raising_left_arm"
            or self.gesture_to_detect is None
        ):
            if part_info["leftWrist"]["y"] < part_info["leftShoulder"]["y"]:
                self.gesture_to_detect = "raising_left_arm"
        if (
            self.gesture_to_detect == "raising_right_arm"
            or self.gesture_to_detect is None
        ):
            if part_info["rightWrist"]["y"] < part_info["rightShoulder"]["y"]:
                self.gesture_to_detect = "raising_right_arm"
        if (
            self.gesture_to_detect == "pointing_to_the_left"
            or self.gesture_to_detect is None
        ):
            if (
                part_info["leftWrist"]["x"] - self.buffer_width
                > part_info["leftShoulder"]["x"]
            ):
                self.gesture_to_detect = "pointing_to_the_left"
        if (
            self.gesture_to_detect == "pointing_to_the_right"
            or self.gesture_to_detect is None
        ):
            if (
                part_info["rightShoulder"]["x"] - self.buffer_width
                > part_info["rightWrist"]["x"]
            ):
                self.gesture_to_detect = "pointing_to_the_right"

        if self.gesture_to_detect is None:
            self.gesture_to_detect = "none"

        userdata.gesture_detected = self.gesture_to_detect

        cv2_gesture_img = cv2_img.msg_to_cv2_img(userdata.img_msg)
        # Add text to the image
        cv2.putText(
            cv2_gesture_img,
            self.gesture_to_detect,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        # Publish the image
        self.debug_publisher.publish(cv2_img.cv2_img_to_msg(cv2_gesture_img))

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
    rospy.init_node("gesture_detection_sm")
    ### Example usage:
    while not rospy.is_shutdown():
        sm = GestureDetectionSM()
        sm.execute()
        gesture_state = PlayMotion(motion_name=sm.userdata.gesture_detected)
        gesture_sm = smach.StateMachine(outcomes=["succeeded", "failed"])
        with gesture_sm:
            smach.StateMachine.add(
                "GESTURE_STATE",
                gesture_state,
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )
        gesture_sm.execute()

    rospy.spin()
