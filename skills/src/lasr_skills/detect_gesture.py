import smach
import rospy
import cv2
import cv2_img
from lasr_vision_msgs.srv import (
    BodyPixKeypointDetection,
    BodyPixKeypointDetectionRequest,
)
from sensor_msgs.msg import Image

from typing import Union


class DetectGesture(smach.State):
    """
    State for detecting gestures.
    """

    def __init__(
        self,
        gesture_to_detect: Union[str, None] = None,
        bodypix_model: str = "resnet50",
        bodypix_confidence: float = 0.1,
        buffer_width: int = 50,
        debug_publisher: str = "/skills/gesture_detection/debug",
    ):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["img_msg"],
            output_keys=["detected_gesture"],
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

        detected_gesture = "none"

        keypoint_info = {
            keypoint.keypoint_name: {"x": keypoint.x, "y": keypoint.y}
            for keypoint in detected_keypoints
        }

        # raising left arm
        if "leftShoulder" in keypoint_info and "leftWrist" in keypoint_info:
            if keypoint_info["leftWrist"]["y"] < keypoint_info["leftShoulder"]["y"]:
                detected_gesture = "raising_left_arm"
        # pointing to the left
        if "leftShoulder" in keypoint_info and "leftWrist" in keypoint_info:
            if (
                keypoint_info["leftWrist"]["x"] - self.buffer_width
                > keypoint_info["leftShoulder"]["x"]
            ):
                detected_gesture = "pointing_to_the_left"
        # raising right arm
        if "rightShoulder" in keypoint_info and "rightWrist" in keypoint_info:
            if keypoint_info["rightWrist"]["y"] < keypoint_info["rightShoulder"]["y"]:
                detected_gesture = "raising_right_arm"
        # pointing to the right
        if "rightShoulder" in keypoint_info and "rightWrist" in keypoint_info:
            if (
                keypoint_info["rightShoulder"]["x"] - self.buffer_width
                > keypoint_info["rightWrist"]["x"]
            ):
                detected_gesture = "pointing_to_the_right"

        if self.gesture_to_detect == "waving":
            if detected_gesture in ["raising_left_arm", "raising_right_arm"]:
                detected_gesture = "waving"

        rospy.loginfo(f"Detected gesture: {detected_gesture}")
        userdata.detected_gesture = detected_gesture

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

        if self.gesture_to_detect is not None:
            return (
                "succeeded" if detected_gesture == self.gesture_to_detect else "failed"
            )
        else:
            return "succeeded"
