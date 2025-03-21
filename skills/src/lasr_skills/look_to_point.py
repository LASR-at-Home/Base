import smach
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import PointHead
from geometry_msgs.msg import Point, PointStamped

from typing import Union


class LookToPoint(smach.State, Node):
    _pointstamped: Union[None, PointStamped]

    def __init__(self, pointstamped: Union[None, PointStamped] = None):
        Node.__init__(self, "look_to_point")
        smach.State.__init__(
            self,
            outcomes=["succeeded", "aborted", "timed_out"],
            input_keys=["pointstamped"] if pointstamped is None else [],
        )

        self._pointstamped = pointstamped

        self.client = ActionClient(
            self, PointHead, "/head_controller/point_head_action"
        )
        self.goal_future = None
        self.result_future = None
        self.get_logger().info("Created State")

        self.client.wait_for_server()

    def execute(self, userdata):
        # Define the goal
        goal = PointHead.Goal(
            pointing_frame="head_2_link",
            pointing_axis=Point(x=1.0, y=0.0, z=0.0),
            max_velocity=1.0,
            target=(
                self._pointstamped
                if self._pointstamped is not None
                else userdata.pointstamped
            ),
        )

        # Send the goal
        self.get_logger().info("Sending goal")
        future = self.client.send_goal_async(
            goal
        )  # can't call send_goal in cb because of deadlock
        rclpy.spin_until_future_complete(self, future)

        # Wait for the result with a timeout of 2 seconds
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if result_future.done():
            state = result_future.result()
            # state = self.client.get_state()
            if state.status == state.SUCCEEDED:
                return "succeeded"
            else:
                return "aborted"
        else:
            # result_future.cancel()
            goal_handle.cancel_goal_async()
            return "timed_out"
