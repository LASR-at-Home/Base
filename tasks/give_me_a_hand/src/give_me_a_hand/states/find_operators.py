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
# from std_msgs.msg import Empty

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from pal_startup_msgs.srv import (
    StartupStart,
    StartupStartRequest,
    StartupStop,
    StartupStopRequest,
)

class FindOperators(smach.StateMachine):
    def __init__(self, table_pose, cabinet_pose):
        super().__init__(
            outcomes=["succeeded", "failed", "escape"],
            input_keys=[],
            output_keys=[],
        )

        with self:
            self.go_to_(self, pose, "ROTATE_360")

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
                    "customer_found": "GO_TO_OPERATORS",
                    "customer_not_found": "escape",
                },
            )

            go_to_(self, customer_approach_pose, "succeeded") # Operator

            # smach.StateMachine.add(
            #     "GO_TO_OPERATORS",
            #     GoToLocation(),
            #     remapping={"location": "customer_approach_pose"},
            #     transitions={"succeeded": "GET_ORDER", "failed": "TRUN_MOVE_BASE"},
            # )

    def go_to_(self, pose, next_state) -> None:
        """Adds the states to go to table area."""

        smach.StateMachine.add(
            f"GO_TO_{next_state}",
            GoToLocation(pose),
            transitions={
                "succeeded": f"{next_state}",
                "failed": f"GO_TO_RECOVERY",
            },
        )

        smach.StateMachine.add(
            "ENABLE_MOVEBASE_MANAGER",
            smach_ros.ServiceState(
            "/pal_startup_control/start",
            StartupStart,
            request=StartupStartRequest("move_base", ""),
            ),
            transitions={
                "succeeded": f"GO_TO_{next_state}",
                "preempted": "failed",
                "aborted": "failed",
            },
        ),