import rospy
import smach
from smach import CBState
from lasr_skills import Say, AdjustCamera, GoToLocation, CheckDoorStatus, DetectDict
from give_me_a_hand.states import (
    Survey, HandleOrder, GetPoses
)

import smach
import smach_ros
from geometry_msgs.msg import Pose
from lasr_skills import AskAndListen, GoToLocation, Rotate, Say, Wait

class FindOperators(smach.StateMachine):
    def __init__(self, table_pose, cabinet_pose):
        super().__init__(
            outcomes=["succeeded", "failed", "escape"],
            input_keys=[],
            output_keys=[],
        )

        with self:
            
            smach.StateMachine.add(
                "HOLD_OBJECT",
                HoldObject(),
                transitions={
                    "succeeded": "DELIVER_OBJECT",
                    "aborted": "DELIVER_OBJECT",
                    "preempted": "DELIVER_OBJECT",
                },
            )

            smach.StateMachine.add(
                "DELIVER_OBJECT",
                DeliverObject(),
                transitions={
                    "succeeded": "SELECT_OBJECT",
                    "aborted": "SELECT_OBJECT",
                    
                },
            )

            smach.StateMachine.add(
                "PUT_OBJECT",
                PutObject(),
                transitions={
                    "succeeded": "SAY_ORDER_DONE",
                    "aborted": "SAY_ORDER_DONE",
                    "preempted": "SAY_ORDER_DONE",
                },
            )