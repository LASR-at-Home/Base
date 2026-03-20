from smach_ros import RosState
import rclpy
from rclpy.action import ActionClient

from control_msgs.action import PointHead
from geometry_msgs.msg import Point, PointStamped

from typing import Union


class LookToPoint(RosState):
    _pointstamped: Union[None, PointStamped]

    def __init__(
        self,
        node,
        pointstamped: Union[None, PointStamped] = None,
    ):
        super().__init__(
            node,
            outcomes=["succeeded", "aborted", "timed_out"],
            input_keys=["pointstamped"] if pointstamped is None else [],
        )

        self._pointstamped = pointstamped

        self.client = ActionClient(
            self.node,
            PointHead,
            "/head_controller/point_head_action",  # TODO: Action server doesnt exist
        )
        self.goal_future = None
        self.result_future = None
        self.node.get_logger().info("LookToPoint - Created State.")

        if not self.client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().warn(
                "Head controller PointHead action server not found. Skipping..."
            )
        else:
            self.node.get_logger().info(
                "Head controller PointHead action server working."
            )

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
        self.node.get_logger().info("Sending goal - PointHead")
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)

        # Wait for the result with a timeout of 2 seconds
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=2.0)

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
