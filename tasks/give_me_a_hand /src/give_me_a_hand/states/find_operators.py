import rospy
import smach
from smach import CBState
from lasr_skills import Say, AdjustCamera, GoToLocation, CheckDoorStatus, DetectDict
from give_me_a_hand.states import (
    Survey, GetPoses
)

import smach
import smach_ros
from geometry_msgs.msg import Pose
from lasr_skills import AskAndListen, GoToLocation, Rotate, Say, Wait
# from std_msgs.msg import Empty

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from pal_startup_msgs.srv import (
    StartupStart,
    StartupStartRequest,
    StartupStop,
    StartupStopRequest,
)

class FindOperators(smach.StateMachine):
    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed", "escape"],
            input_keys=[],
            output_keys=['customer_approach_pose'],
        )

        with self:
            smach.StateMachine.add(
                "SAY_FINISHED",
                Say(text="It is too noisy, can't find you with sound. Please keep wave the hand to call me"),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "succeeded",
                },
            )

            smach.StateMachine.add(
                "ROTATE_360",
                Rotate(angle=360.0),
                transitions={"succeeded": "GET_POSES", "failed": "ROTATE_360"},
            )

            smach.StateMachine.add(
                "GET_POSES", GetPoses(), transitions={"succeeded": "SURVEY"}
            )

            smach.StateMachine.add(
                "SURVEY",
                Survey((-71.0, 71.0), 10),
                transitions={
                    "customer_found": "succeeded",
                    "customer_not_found": "escape",
                },
            )