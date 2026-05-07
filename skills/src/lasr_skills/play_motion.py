from typing import Union

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus

from smach_ros import RosState

from play_motion2_msgs.action import PlayMotion2

# https://github.com/pal-robotics/play_motion2


class PlayMotion(RosState):

    def __init__(self, node: Node, motion_name: Union[str, None] = None):
        super().__init__(
            node=node,
            outcomes=["succeeded", "aborted", "preempted"],
            input_keys=["motion_name"] if motion_name is None else [],
        )

        self.motion_name = motion_name

        self.action_client = ActionClient(
            node,
            PlayMotion2,
            "/play_motion2",
        )

    def execute(self, ud):
        if self.preempt_requested():
            self.service_preempt()
            return "preempted"

        self.node.get_logger().info("Waiting for /play_motion2 action server...")

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("PlayMotion2 action server not available")
            return "aborted"

        goal = self.create_goal(ud)

        send_goal_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_goal_future)

        goal_handle = send_goal_future.result()

        if goal_handle is None:
            self.node.get_logger().error("Failed to send PlayMotion goal")
            return "aborted"

        if not goal_handle.accepted:
            self.node.get_logger().warn("PlayMotion goal rejected")
            return "aborted"

        self.node.get_logger().info("PlayMotion goal accepted")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        result_response = result_future.result()

        if result_response is None:
            self.node.get_logger().error("Failed to get PlayMotion result")
            return "aborted"

        return self.handle_result(
            result_response.status,
            result_response.result,
        )

    def create_goal(self, ud):
        goal = PlayMotion2.Goal()

        if self.motion_name is None:
            goal.motion_name = ud.motion_name
        else:
            goal.motion_name = self.motion_name

        goal.skip_planning = False

        self.node.get_logger().warn(f"PlayMotion Goal sent: {goal}")

        return goal

    def handle_result(self, status, result):
        self.node.get_logger().warn(f"PlayMotion Result: {result}")

        if status == GoalStatus.STATUS_SUCCEEDED:
            return "succeeded"

        if status == GoalStatus.STATUS_CANCELED:
            return "preempted"

        return "aborted"
