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
from lasr_skills import LookAtPerson, LookToPoint
from typing import List, Union
from geometry_msgs.msg import Point, PointStamped
from lasr_vision_msgs.srv import Recognise, RecogniseRequest
from lasr_skills.vision import GetPointCloud
from cv2_pcl import pcl_to_img_msg
from std_msgs.msg import Header

import actionlib
from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import Point


class FindAndLookAt(smach.StateMachine):
    class GetLookPoint(smach.State):
        def __init__(self, look_positions: List[Point]):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=[],
                output_keys=["look_positions"],
            )
            self.look_positions = look_positions

        def execute(self, userdata):
            userdata.look_positions = self.look_positions
            return "succeeded"

    class GetPoint(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["point_index", "look_positions"],
                output_keys=["pointstamped"],
            )
            self.look_at_pub = actionlib.SimpleActionClient(
                "/head_controller/point_head_action", PointHeadAction
            )

        def execute(self, userdata):
            point = userdata.look_positions[userdata.point_index]
            userdata.pointstamped = PointStamped(point=point)
            target = PointStamped()
            target.header = Header()
            target.header.frame_id = "head_2_link"
            target.point = point
            self.look_at_pub.wait_for_server()
            goal = PointHeadGoal()
            goal.pointing_frame = "head_2_link"
            goal.pointing_axis = Point(1.0, 0.0, 0.0)
            goal.max_velocity = 1.0
            goal.target = target
            self.look_at_pub.send_goal(goal)
            self.look_at_pub.wait_for_result()
            print(self.look_at_pub.get_state())
            print(f"Looking at {point}")
            rospy.sleep(1.0)
            return "succeeded"

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
                # Point(0.0, 0.0, 0.0),
                Point(1.0, -1.0, 0.0),
                Point(-1.0, -1.0, 0.0),
            ]

        all_look_positions = look_positions
        print(all_look_positions)

        with self:
            # look straight
            # check if name is in the frame
            # if not look left
            # if not look right
            # if not fail
            smach.StateMachine.add(
                "GET_LOOK_POINT",
                self.GetLookPoint(all_look_positions),
                transitions={"succeeded": "LOOK_ITERATOR", "failed": "failed"},
            )
            look_iterator = smach.Iterator(
                outcomes=["succeeded", "failed"],
                it=lambda: range(len(all_look_positions)),
                it_label="point_index",
                input_keys=["look_positions", "dataset", "confidence", "guest_name"],
                output_keys=[],
                exhausted_outcome="failed",
            )
            with look_iterator:
                container_sm = smach.StateMachine(
                    outcomes=["succeeded", "failed", "continue"],
                    input_keys=[
                        "point_index",
                        "look_positions",
                        "dataset",
                        "confidence",
                        "guest_name",
                    ],
                    output_keys=["pointstamped"],
                )

                with container_sm:
                    smach.StateMachine.add(
                        "GET_POINT",
                        self.GetPoint(),
                        transitions={"succeeded": "GET_IMAGE", "failed": "failed"},
                        remapping={"pointstamped": "pointstamped"},
                    )
                    # smach.StateMachine.add(
                    #     "LOOK_TO_POINT",
                    #     LookToPoint(),
                    #     transitions={
                    #         "succeeded": "GET_IMAGE",
                    #         "aborted": "failed",
                    #         "preempted": "failed",
                    #     },
                    # )
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
                        remapping={
                            "pcl_msg": "pcl_msg",
                            "detections": "deepface_detection",
                        },
                    )
                    smach.StateMachine.add(
                        "LOOK_AT_PERSON",
                        LookAtPerson(filter=True),
                        transitions={
                            "succeeded": "succeeded",
                            "failed": "failed",
                        },
                    )
                look_iterator.set_contained_state(
                    "CONTAINER_STATE", container_sm, loop_outcomes=["continue"]
                )
            smach.StateMachine.add(
                "LOOK_ITERATOR",
                look_iterator,
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )


if __name__ == "__main__":
    rospy.init_node("test_find_and_look_at")

    sm = FindAndLookAt(
        "Nicole",
        [
            Point(-0.5, 0.0, 1.0),
            Point(0.0, 0.0, 1.0),
            Point(-0.5, 0.0, 1.0),
        ],
    )
    sm.userdata.guest_name = "Nicole"
    sm.userdata.dataset = "receptionist"
    sm.userdata.confidence = 0.5

    outcome = sm.execute()

    rospy.loginfo(f"Outcome: {outcome}")
