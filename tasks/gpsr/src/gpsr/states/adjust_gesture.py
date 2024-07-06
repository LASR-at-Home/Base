#!/usr/bin/env python3
import rospy
import smach
from adjust_base import AdjustBase
from adjust_torso import TorsoHeight
from adjust_head import AdjustHeadTilt
from lasr_vision_msgs.srv import BodyPixDetection, BodyPixDetectionRequest
from lasr_vision_msgs.msg import BodyPixMaskRequest
from lasr_skills.vision import GetImage


class DetectNose(smach.State):
    """
    State for detecting gestures.
    # adjust base
    # detect nose --> if it in the image--> head up
    # --> if not adjust the torso until we can see the nose --> adjust head
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["move_up", "move_down", "stay_same", "no_person", "no_nose"],
            input_keys=["img_msg"],
        )

        self.body_pix_client = rospy.ServiceProxy("/bodypix/detect", BodyPixDetection)

    def execute(self, userdata):

        body_pix_masks = BodyPixMaskRequest()
        body_pix_masks.parts = ["nose"]
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
        if not poses:
            rospy.logerr("No poses available in the response. No person detected.")
            return "no_person"

        for pose in poses:
            for keypoint in pose.keypoints:
                if pose is None or pose.keypoints is None:
                    rospy.logerr("No nose available in the response.")
                    return "no_nose"  # Skip this iteration if data is missing
                else:
                    part_info[keypoint.part] = {
                        "x": keypoint.xy[0],
                        "y": keypoint.xy[1],
                        "score": keypoint.score,
                    }

        # img (640 480)  origin of coordinate: upperleft

        if (part_info["nose"]["x"]) < 220:
            return "move_down"
        elif (part_info["nose"]["x"]) > 260:
            return "move_up"
        else:
            return "stay_same"


class AdjustGesture(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])
        with self:
            smach.StateMachine.add(
                "ADJUST_BASE",
                AdjustBase(),
                transitions={"succeeded": "GET_IMAGE", "failed": "failed"},
            )

            smach.StateMachine.add(
                "GET_IMAGE",
                GetImage(),
                transitions={"succeeded": "DETECT_NOSE", "failed": "failed"},
            )

            smach.StateMachine.add(
                "DETECT_NOSE",
                DetectNose(),
                transitions={
                    "move_up": "HEAD_MOVE_UP",
                    "move_down": "HEAD_MOVE_DOWN",
                    "stay_same": "suceeded",
                    "no_nose": "ADJUST_TORSO",
                    "no_person": "GET_IMAGE",
                },
            )

            smach.StateMachine.add(
                "HEAD_MOVE_UP",
                AdjustHeadTilt(0.1, 1),
                transitions={"succeeded": "GET_IMAGE", "failed": "failed"},
            )

            smach.StateMachine.add(
                "HEAD_MOVE_DOWN",
                AdjustHeadTilt(0.1, 1),
                transitions={"succeeded": "GET_IMAGE", "failed": "failed"},
            )

            smach.StateMachine.add(
                "ADJUST_TORSO",
                TorsoHeight(-0.1, 1),
                transitions={"succeeded": "GET_IMAGE", "failed": "failed"},
            )


if __name__ == "__main__":
    rospy.init_node("adjust_gesture")
    while not rospy.is_shutdown():
        sm = AdjustGesture()
        sm.execute()
    rospy.spin()
