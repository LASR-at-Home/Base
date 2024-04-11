import smach_ros
import smach
import rospy
import rosservice
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header

from typing import Union

from lasr_skills import PlayMotion

PUBLIC_CONTAINER = False

try:
    from pal_startup_msgs.srv import (
        StartupStart,
        StartupStop,
        StartupStartRequest,
        StartupStopRequest,
    )
except ModuleNotFoundError:
    PUBLIC_CONTAINER = True


class GoToLocation(smach.StateMachine):

    def __init__(self, location: Union[Pose, None] = None):
        super(GoToLocation, self).__init__(outcomes=["succeeded", "failed"])

        with self:
            smach.StateMachine.add(
                "LOWER_BASE",
                PlayMotion("pre_navigation"),
                transitions={
                    "succeeded": "ENABLE_HEAD_MANAGER",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            IS_SIMULATION = (
                "/pal_startup_control/start" in rosservice.get_service_list()
            )

            if not IS_SIMULATION:

                if PUBLIC_CONTAINER:
                    rospy.logwarn(
                        "You are using a public container. The head manager will not be stopped during navigation."
                    )
                else:
                    smach.StateMachine.add(
                        smach_ros.SimpleServiceState(
                            "/pal_startup_control/start",
                            StartupStart,
                            request=StartupStartRequest("head_manager", ""),
                        ),
                    )

            if location is not None:
                smach.StateMachine.add(
                    "GO_TO_LOCATION",
                    smach_ros.SimpleActionState(
                        "move_base",
                        MoveBaseAction,
                        goal=MoveBaseGoal(
                            target_pose=PoseStamped(
                                pose=location, header=Header(frame_id="map")
                            )
                        ),
                    ),
                    transitions={
                        "succeeded": "DISABLE_HEAD_MANAGER",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )
            else:
                smach.StateMachine.add(
                    "GO_TO_LOCATION",
                    smach_ros.SimpleActionState(
                        "move_base",
                        MoveBaseAction,
                        goal_cb=lambda ud, _: MoveBaseGoal(
                            target_pose=PoseStamped(
                                pose=ud.location, header=Header(frame_id="map")
                            )
                        ),
                        input_keys=["location"],
                    ),
                    transitions={
                        "succeeded": "DISABLE_HEAD_MANAGER",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )

            if not IS_SIMULATION:

                if PUBLIC_CONTAINER:
                    rospy.logwarn(
                        "You are using a public container. The head manager will not be start following navigation."
                    )
                else:
                    smach.StateMachine.add(
                        "DISABLE_HEAD_MANAGER",
                        smach_ros.SimpleServiceState(
                            "/pal_startup_control/stop",
                            StartupStop,
                            request=StartupStopRequest("head_manager"),
                        ),
                        transitions={
                            "succeeded": "succeeded",
                            "aborted": "succeeded",
                            "preempted": "succeeded",
                        },
                    )
