#!/usr/bin/env python3
import smach
import rospy
from lasr_skills.vision import GetImage
from lasr_vision_msgs.srv import BodyPixDetection, BodyPixDetectionRequest
from lasr_vision_msgs.msg import BodyPixMaskRequest


class DetectGesture(smach.State):
    """
    State for detecting gestures.
    """

    def __init__(self, gesture_to_detect: str = "person raising their left arm"):
        self.gesture_to_detect = gesture_to_detect
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["img_msg"],
            output_keys=["gesture_detected"],
        )

        self.body_pix_client = rospy.ServiceProxy("/bodypix/detect", BodyPixDetection)

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

        if self.gesture_to_detect == "person raising their left arm":
            if part_info["leftWrist"]["y"] < part_info["leftShoulder"]["y"]:
                userdata.gesture_detected = True
            else:
                userdata.gesture_detected = False
        elif self.gesture_to_detect == "person raising their right arm":
            if part_info["rightWrist"]["y"] < part_info["rightShoulder"]["y"]:
                userdata.gesture_detected = True
            else:
                userdata.gesture_detected = False
        else:
            raise ValueError("Invalid gesture to detect")

        return "succeeded"


class GestureDetectionSM(smach.StateMachine):
    """
    State machine for detecting gestures.
    """

    def __init__(self, gesture_to_detect: str = "person raising their left arm"):
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
    while not rospy.is_shutdown():
        sm = GestureDetectionSM()
        sm.execute()
        print("Raising Left Hand Detected:", sm.userdata.gesture_detected)

    rospy.spin()
