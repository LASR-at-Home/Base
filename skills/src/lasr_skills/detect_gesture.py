#!/usr/bin/env python3
from typing import Optional, List
import smach
import rospy
import cv2
import cv2_img
from lasr_skills.vision import GetCroppedImage, GetImage
from lasr_vision_msgs.srv import (
    BodyPixKeypointDetection,
    BodyPixKeypointDetectionRequest,
)
from sensor_msgs.msg import Image


class DetectGesture(smach.State):
    """
    State for detecting gestures.
    """

    def __init__(
        self,
        gesture_to_detect: Optional[str] = None,
        bodypix_model: str = "resnet50",
        bodypix_confidence: float = 0.1,
        buffer_width: int = 50,
        debug_publisher: str = "/skills/gesture_detection/debug",
    ):
        """Optionally stores the gesture to detect. If None, it will infer the gesture from the keypoints."""
        smach.State.__init__(
            self,
            outcomes=["succeeded", "missing_keypoints", "failed"],
            input_keys=["img_msg"],
            output_keys=["gesture_detected"],
        )
        self.gesture_to_detect = gesture_to_detect
        self.bodypix_client = rospy.ServiceProxy(
            "/bodypix/keypoint_detection", BodyPixKeypointDetection
        )
        self.bodypix_model = bodypix_model
        self.bodypix_confidence = bodypix_confidence
        self.debug_publisher = rospy.Publisher(debug_publisher, Image, queue_size=1)
        self.buffer_width = buffer_width
        self.required_keypoints = [
            "leftWrist",
            "leftShoulder",
            "rightWrist",
            "rightShoulder",
        ]

    def execute(self, userdata):

        req = BodyPixKeypointDetectionRequest()
        req.image_raw = userdata.img_msg
        req.dataset = self.bodypix_model
        req.confidence = self.bodypix_confidence
        req.keep_out_of_bounds = False

        try:
            res = self.bodypix_client(req)
        except Exception as e:
            print(e)
            return "failed"

        detected_keypoints = res.keypoints

        keypoint_info = {
            keypoint.keypoint_name: {"x": keypoint.x, "y": keypoint.y}
            for keypoint in detected_keypoints
        }

        if "leftShoulder" in keypoint_info and "leftWrist" in keypoint_info:
            if (
                self.gesture_to_detect == "raising_left_arm"
                or self.gesture_to_detect is None
            ):
                if keypoint_info["leftWrist"]["y"] < keypoint_info["leftShoulder"]["y"]:
                    self.gesture_to_detect = "raising_left_arm"
            if (
                self.gesture_to_detect == "pointing_to_the_left"
                or self.gesture_to_detect is None
            ):
                if (
                    keypoint_info["leftWrist"]["x"] - self.buffer_width
                    > keypoint_info["leftShoulder"]["x"]
                ):
                    self.gesture_to_detect = "pointing_to_the_left"

        if (
            "rightShoulder" in keypoint_info
            and "rightWrist" in keypoint_info
            and self.gesture_to_detect is None
        ):
            print(keypoint_info["rightShoulder"]["x"], keypoint_info["rightWrist"]["x"])
            if (
                self.gesture_to_detect == "raising_right_arm"
                or self.gesture_to_detect is None
            ):
                if (
                    keypoint_info["rightWrist"]["y"]
                    < keypoint_info["rightShoulder"]["y"]
                ):
                    self.gesture_to_detect = "raising_right_arm"
            if (
                self.gesture_to_detect == "pointing_to_the_right"
                or self.gesture_to_detect is None
            ):
                if (
                    keypoint_info["rightShoulder"]["x"] - self.buffer_width
                    > keypoint_info["rightWrist"]["x"]
                ):
                    self.gesture_to_detect = "pointing_to_the_right"

        if self.gesture_to_detect is None:
            self.gesture_to_detect = "none"

        rospy.loginfo(f"Detected gesture: {self.gesture_to_detect}")
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
                GetCroppedImage("person", "closest", use_mask=True),
                transitions={"succeeded": "BODY_PIX_DETECTION", "failed": "failed"},
            )

            smach.StateMachine.add(
                "BODY_PIX_DETECTION",
                DetectGesture(gesture_to_detect=self.gesture_to_detect),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                    "missing_keypoints": "failed",
                },
            )


if __name__ == "__main__":
    rospy.init_node("gesture_detection_sm")
    ### Example usage:
    while not rospy.is_shutdown():
        sm = GestureDetectionSM()
        sm.execute()

    rospy.spin()
