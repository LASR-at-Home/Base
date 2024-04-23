#!/usr/bin/env python3

import numpy as np
import smach_ros
from control_msgs.msg import PointHeadGoal, PointHeadAction
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
import smach
from vision import GetImage, ImageMsgToCv2
from lasr_vision_msgs.srv import BodyPixDetection, BodyPixDetectionRequest
from lasr_vision_msgs.msg import BodyPixMaskRequest
from lasr_skills import LookToPoint
import cv2
import rospy
from sensor_msgs.msg import Image
from cv2_img import cv2_img_to_msg, msg_to_cv2_img

# find persons eyes
# if you cant find increase the torso height
# if you can find the eyes call look to point

class CheckEyes(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"], input_keys=["parts"])
        self.img_pub = rospy.Publisher("/debug/image", Image, queue_size=1)

    def execute(self, userdata):
        # ge the coordinates of the eyes

        if userdata.parts is None:
            return "failed"
        for pose in userdata.poses:
            if pose is None:
                return "failed"

        # visualise on image
        img = msg_to_cv2_img(userdata.img_msg)

        cv2.circle(userdata.img, (pose[0], pose[1]), 5, (0, 255, 0), -1)
        img = cv2_img_to_msg(img)

        self.img_pub.publish(img)


        return "succeeded"

class IncreaseTorsoHeight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

    def execute(self, userdata):
        return "succeeded"


class LookAtPerson(smach.StateMachine):

    def __init__(self):
        super(LookAtPerson, self).__init__(outcomes=["succeeded", "failed"], input_keys=[], output_keys=[ "masks", "poses"])


        with self:
                smach.StateMachine.add(
                    "GET_IMAGE",
                    GetImage(),
                    transitions={
                        "succeeded": "SEGMENT_PERSON",
                    },
                    remapping={"img_msg": "img_msg"},
                )
                eyes = BodyPixMaskRequest()
                eyes.parts = ['left_eye', 'right_eye']
                masks = [eyes]

                smach.StateMachine.add(
                    "SEGMENT_PERSON",
                    smach_ros.ServiceState(
                        "/bodypix/detect",
                        BodyPixDetection,
                        request_cb=lambda ud, _: BodyPixDetectionRequest(ud.img_msg, "resnet50", 0.7, masks),
                        response_slots=['masks', 'poses'],
                        input_keys=["img_msg"],
                        output_keys=["masks", "poses"],
                    ),
                    transitions={
                        "succeeded": "succeeded",
                        # "succeeded": "LOOK_TO_POINT",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                    remapping={"img_msg": "img_msg", "masks": "masks", "poses": "poses"},
                )
                # smach.StateMachine.add(
                #     "CHECK_EYES",
                #     CheckEyes(),
                #     transitions={
                #         "succeeded": "LOOK_TO_POINT",
                #         "failed": "INCREASE_TORSO_HEIGHT",
                #     },
                # )
                # smach.StateMachine.add(
                #     "INCREASE_TORSO_HEIGHT",
                #     IncreaseTorsoHeight(),
                #     transitions={
                #         "succeeded": "SEGMENT_PERSON",
                #         "failed": "failed",
                #     },
                # )
                # smach.StateMachine.add(
                #     "LOOK_TO_POINT",
                #     LookToPoint(),
                #     transitions={
                #         "succeeded": "succeeded",
                #         "failed": "failed",
                #     },
                # )


if __name__ == "__main__":
    rospy.init_node("look_at_person")
    sm = LookAtPerson()
    outcome = sm.execute()
    rospy.loginfo(outcome)
    # print the output keys
    print()
    print("Output keys:")
    print(sm.userdata.keys())
    print(sm.userdata.poses)
    rospy.spin()











