from typing import Union

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus

from smach_ros import RosState, SimpleActionState

from play_motion2_msgs.action import PlayMotion2

# https://github.com/pal-robotics/play_motion2


class PlayMotion(SimpleActionState):
    def __init__(self, node, motion_name):
        super().__init__(
            node=node,
            action_name="/play_motion2",
            action_spec=PlayMotion2,
            goal_cb=self._create_goal,
            result_cb=self._result_handle,
            input_keys=["motion_name"] if motion_name is None else [],
        )

        self.motion_name = motion_name

    def _create_goal(self, ud, goal):
        goal.motion_name = (
            ud.motion_name if self.motion_name is None else self.motion_name
        )
        goal.skip_planning = False

        self.node.get_logger().info(f"GIVING GOAL of {goal.motion_name}")

        return goal

    def _result_handle(self, ud, result_status, result):
        self.node.get_logger().info(f"Received result with status: {result_status}")
