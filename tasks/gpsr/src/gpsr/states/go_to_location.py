import smach_ros

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header

from typing import Union


class GoToLocation(smach_ros.SimpleActionState):
    def __init__(self, location: Union[Pose, None] = None):
        if location is not None:
            super(GoToLocation, self).__init__(
                "move_base",
                MoveBaseAction,
                goal=MoveBaseGoal(
                    target_pose=PoseStamped(
                        pose=location, header=Header(frame_id="map")
                    )
                ),
            )
        else:
            super(GoToLocation, self).__init__(
                "move_base",
                MoveBaseAction,
                goal_cb=lambda ud, _: MoveBaseGoal(
                    target_pose=PoseStamped(
                        pose=ud.location, header=Header(frame_id="map")
                    )
                ),
                input_keys=["location"],
            )
