#!/usr/bin/env python
from typing import List, Tuple

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Point, PointStamped, Pose
from lasr_skills import Say, GoToLocation, AskAndListen
from lasr_vision_msgs.srv import Recognise
from give_me_a_hand.states import WaitDoorOpen, HandoverAndDeliver
from give_me_a_hand.states import CommunicateOperators#, HandoverAndDeliver, GraspAndPut
from shapely.geometry import Polygon
from std_msgs.msg import Empty, Header



class GiveMeAHand(smach.StateMachine):
    def __init__(
        self,
    ):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:
            # def wait_cb(ud, msg):
            #     rospy.loginfo("Received start signal")
            #     return False

            # smach.StateMachine.add(
            #     "WAIT_START",
            #     smach_ros.MonitorState(
            #         "/give_me_a_hand/start",
            #         Empty,
            #         wait_cb,
            #     ),
            #     transitions={
            #         "valid": "WAIT_START",
            #         "invalid": "SAY_START",
            #         "preempted": "WAIT_START",
            #     },
            # )

            # smach.StateMachine.add(
            #     "SAY_START",
            #     Say(text="Ready to start"),
            #     transitions={
            #         "succeeded": "GO_TO_WAITING_AREA",
            #         "aborted": "GO_TO_WAITING_AREA",
            #         "preempted": "GO_TO_WAITING_AREA",
            #     },
            # )

            # # self.go_to_waiting_area() #may be we don't need this

            # """
            # WaitDoorOpen
            # """

            smach.StateMachine.add(
                "SAY_WAIT",
                Say(text="Waiting for door to open"),
                transitions={
                    "succeeded": "WAIT_DOOR_OPEN",
                    "aborted": "WAIT_DOOR_OPEN",
                    "preempted": "WAIT_DOOR_OPEN",
                },
            )

            smach.StateMachine.add(
                "WAIT_DOOR_OPEN",
                WaitDoorOpen(),
                transitions={
                    "succeeded": "GO_TO_OPERATORS",
                    "aborted": "GO_TO_OPERATORS",
                    "preempted": "GO_TO_OPERATORS",
                },
            )

            """
            FIND_OPERATORS_LOOP
            """

            kitchen_area = Pose()

            kitchen_area.position.x = -2.4909288154765634
            kitchen_area.position.y = -0.06129342248943135
            kitchen_area.position.z = 0.0
            kitchen_area.orientation.x = 0.0
            kitchen_area.orientation.y = 0.0
            kitchen_area.orientation.z = -0.22060310817672235
            kitchen_area.orientation.w = 0.9753636596996883

            smach.StateMachine.add(
                f"GO_TO_OPERATORS",
                GoToLocation(kitchen_area),
                transitions={
                    "succeeded": f"ASK_AND_LISTEN",
                    "failed": f"GO_TO_OPERATORS",
                },
            )

            # smach.StateMachine.add(
            #     "FIND_OPERATORS",
            #     FindOperators(),
            #     transitions={
            #         "succeeded": "GO_TO_OPERATORS",
            #         "failed": "GO_TO_OPERATORS",
            #         "escape": "SAY_FINISHED",
            #     },
            # )

            # smach.StateMachine.add(
            #     f"GO_TO_OPERATORS",
            #     GoToLocation(),
            #     remapping={"location": "customer_approach_pose"},
            #     transitions={
            #         "succeeded": f"COMMUNICATE_OPERATOR",
            #         "failed": f"COMMUNICATE_OPERATOR",
            #     },
            # )

            # smach.StateMachine.add(
            #     "ENABLE_MOVEBASE_MANAGER",
            #     smach_ros.ServiceState(
            #     "/pal_startup_control/start",
            #     StartupStart,
            #     request=StartupStartRequest("move_base", ""),
            #     ),
            #     transitions={
            #         "succeeded": f"GO_TO_OPERATORS",
            #         "preempted": "failed",
            #         "aborted": "failed",
            #     },
            # )

            smach.StateMachine.add(
                "ASK_AND_LISTEN",
                AskAndListen(f"Hi, please tell me your command."),
                transitions={
                    "succeeded": "COMMUNICATE_OPERATOR",
                    "failed": "COMMUNICATE_OPERATOR",
                },
            )

            smach.StateMachine.add(
                "COMMUNICATE_OPERATOR",
                CommunicateOperators(),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "succeeded",
                },
            )

            # smach.StateMachine.add(
            #     "GRASP_AND_PLACE",
            #     HandoverAndDeliver(),
            #     transitions={
            #         "succeeded": "GO_TO_OPERATORS",
            #         "aborted": "GO_TO_OPERATORS",
            #         "preempted": "GO_TO_OPERATORS",
            #     },
            # )

            # """
            # Finish
            # """

            # smach.StateMachine.add(
            #     "SAY_FINISHED",
            #     Say(text="I am done."),
            #     transitions={
            #         "succeeded": "succeeded",
            #         "aborted": "failed",
            #         "preempted": "succeeded",
            #     },
            # )