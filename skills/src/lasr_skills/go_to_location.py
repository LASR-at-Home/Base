import os
from typing import Union

import rosparam
import rospkg
import rospy
import smach
import smach_ros
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from lasr_skills import PlayMotion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Header
from std_srvs.srv import Empty
from pal_startup_msgs.srv import (
    StartupStart,
    StartupStartRequest,
    StartupStop,
    StartupStopRequest,
)


class CheckRetry(smach.State):

    _retry_attempts: int

    def __init__(self, retry_attempts: int):
        super(CheckRetry, self).__init__(
            outcomes=["retry", "done"],
            input_keys=["retry_count"],
            output_keys=["retry_count"],
        )
        self._retry_attempts = retry_attempts

    def execute(self, userdata):
        if userdata.retry_count < self._retry_attempts:
            userdata.retry_count += 1
            return "retry"
        else:
            return "done"


class ClearCostMaps(smach.State):
    """Clears the costmaps of the move_base node."""

    _clear_costmaps: rospy.ServiceProxy

    def __init__(self):
        super(ClearCostMaps, self).__init__(
            outcomes=["succeeded", "failed"],
        )
        self._clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
        self._clear_costmaps.wait_for_service()

    def execute(self, userdata):
        try:
            self._clear_costmaps()
            return "succeeded"
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to clear costmaps: {e}")
            return "failed"


class GoToLocation(smach.StateMachine):
    def __init__(
        self,
        location: Union[Pose, None] = None,
        location_param: Union[str, None] = None,
        safe_navigation: bool = True,
        retry_attempts: int = 0,
    ):
        if location is not None or location_param is not None:
            super(GoToLocation, self).__init__(
                outcomes=["succeeded", "failed"], input_keys=["retry_count"]
            )
        else:
            super(GoToLocation, self).__init__(
                outcomes=["succeeded", "failed"], input_keys=["location", "retry_count"]
            )

        if safe_navigation:
            r = rospkg.RosPack()
            els = rosparam.load_file(
                os.path.join(r.get_path("lasr_skills"), "config", "motions.yaml")
            )
            for param, ns in els:
                rosparam.upload_params(ns, param)
        self._retry_attempts = retry_attempts
        with self:
            self.userdata.retry_count = 0
            if safe_navigation:
                smach.StateMachine.add(
                    "LOWER_BASE",
                    PlayMotion("pre_navigation"),
                    transitions={
                        "succeeded": "ENABLE_HEAD_MANAGER",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )

                smach.StateMachine.add(
                    "ENABLE_HEAD_MANAGER",
                    smach_ros.ServiceState(
                        "/pal_startup_control/start",
                        StartupStart,
                        request=StartupStartRequest("head_manager", ""),
                    ),
                    transitions={
                        "succeeded": "GO_TO_LOCATION",
                        "preempted": "failed",
                        "aborted": "failed",
                    },
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
                            "aborted": "CHECK_RETRY",
                            "preempted": "CHECK_RETRY",
                        },
                    )
                elif location_param is not None:
                    smach.StateMachine.add(
                        "GO_TO_LOCATION",
                        smach_ros.SimpleActionState(
                            "move_base",
                            MoveBaseAction,
                            goal=MoveBaseGoal(
                                target_pose=PoseStamped(
                                    pose=Pose(
                                        position=Point(
                                            **rospy.get_param(
                                                f"{location_param}/position"
                                            )
                                        ),
                                        orientation=Quaternion(
                                            **rospy.get_param(
                                                f"{location_param}/orientation"
                                            )
                                        ),
                                    ),
                                    header=Header(frame_id="map"),
                                )
                            ),
                        ),
                        transitions={
                            "succeeded": "DISABLE_HEAD_MANAGER",
                            "aborted": "CHECK_RETRY",
                            "preempted": "CHECK_RETRY",
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
                            "aborted": "CHECK_RETRY",
                            "preempted": "CHECK_RETRY",
                        },
                    )

                smach.StateMachine.add(
                    "CHECK_RETRY",
                    CheckRetry(self._retry_attempts),
                    transitions={
                        "retry": "CLEAR_COST_MAPS",
                        "done": "RAISE_BASE",
                    },
                    remapping={"retry_count": "retry_count"},
                )

                smach.StateMachine.add(
                    "DISABLE_HEAD_MANAGER",
                    smach_ros.ServiceState(
                        "/pal_startup_control/stop",
                        StartupStop,
                        request=StartupStopRequest("head_manager"),
                    ),
                    transitions={
                        "succeeded": "RAISE_BASE",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )

                smach.StateMachine.add(
                    "RAISE_BASE",
                    PlayMotion("post_navigation"),
                    transitions={
                        "succeeded": "succeeded",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )
