#!/usr/bin/env python3
# # looks at one direction left
# ~recognise
# returns
# bbox
# for eveyrone known in the fram e
# # greps the host(name) if it can see then otherwise fail
# look
# at
# them
# got
# to
# next
# motion
# of
# failed
#
# #
# # add speeing look once recognise given
#
# in look
# to
# point
# 192


""" 
State machine for the nicole task. It finds a person by given name and then looks at them.
"""
import rospy
import smach_ros
import smach
from lasr_skills.look_at_person import LookAtPerson
from typing import List, Union
from geometry_msgs.msg import Point
from lasr_vision_msgs.srv import Recognise, RecogniseRequest
from lasr_skills.vision import GetPointCloud
from cv2_pcl import pcl_to_img_msg


class FindAndLookAt(smach.StateMachine):
    def __init__(
        self,
        guest_name: str,
        look_positions: Union[List[Point], None] = None,
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_name", "dataset", "confidence"],
            output_keys=[],
        )

        if look_positions is None:
            all_look_positions: List[Point] = []
            # look straight, left, right
            look_positions = [
                Point(0.0, 0.0, 0.0),
                Point(1.0, -1.0, 0.0),
                Point(-1.0, -1.0, 0.0),
            ]
        else:
            all_look_positions = look_positions

        with self:
            # look straight
            # check if name is in the frame
            # if not look left
            # if not look right
            # if not fail
            smach.StateMachine.add(
                "GET_IMAGE",
                GetPointCloud("/xtion/depth_registered/points"),
                transitions={
                    "succeeded": "RECOGNISE",
                },
                remapping={"pcl_msg": "pcl_msg"},
            )

            smach.StateMachine.add(
                "RECOGNISE",
                smach_ros.ServiceState(
                    "/recognise",
                    Recognise,
                    input_keys=["pcl_msg", "dataset", "confidence"],
                    request_cb=lambda ud, _: RecogniseRequest(
                        image_raw=pcl_to_img_msg(ud.pcl_msg),
                        dataset=ud.dataset,
                        confidence=ud.confidence,
                    ),
                    response_slots=["detections"],
                    output_keys=["detections"],
                ),
                transitions={
                    "succeeded": "LOOK_AT_PERSON",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"pcl_msg": "pcl_msg", "detections": "deepface_detection"},
            )
            smach.StateMachine.add(
                "LOOK_AT_PERSON",
                LookAtPerson(filter=True),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )


if __name__ == "__main__":
    rospy.init_node("test_find_and_look_at")

    sm = FindAndLookAt(
        "Nicole",
        [
            Point(0.0, 0.0, 0.0),
            Point(1.0, -1.0, 0.0),
            Point(-1.0, -1.0, 0.0),
        ],
    )
    sm.userdata.guest_name = "Nicole"
    sm.userdata.dataset = "receptionist"
    sm.userdata.confidence = 0.5

    outcome = sm.execute()

    rospy.loginfo(f"Outcome: {outcome}")
