#!/usr/bin/env/python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import GoalHandle
from rclpy.client import Client
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

from lasr_person_following.action import Follow
from lasr_person_following import PersonFollower

from typing import Optional, List


class PersonFollowingServer(Node):
    _dynamic_costmap: Client
    _dynamic_velocity: Client
    _dynamic_recovery: Client
    _dynamic_local_costmap: Client

    _follower: PersonFollower
    _server: ActionServer

    def __init__(self) -> None:
        super().__init__("person_following_server")
        self._set_nav2_parameters = self.create_client(
            SetParameters, "/nav2/set_parameters"
        )
        while not self._set_nav2_parameters.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "/nav2/set_parameters not available, waiting again..."
            )

        self._update_params()
        self.get_logger().info("Updated params")

        self._follower = PersonFollower()
        self._action_server = ActionServer(
            self, Follow, "follow_person", self._execute_cb
        )
        self.get_logger().info("Action server started")

    def _execute_cb(self, _: GoalHandle) -> Follow.Result:
        print("Executing follow_person action")
        while not self._follower.begin_tracking():
            self.get_logger().warning("No people found, retrying...")

        self.get_logger().warning(
            "PersonFollowingServer does not handle preempting or cancelling yet, so the execution will continue until the person is lost."
        )
        self.get_logger().warning(
            "PersonFollowingServer does not provide feedback yet."
        )

        self._follower.follow()

        return Follow.Result()

    def _update_params(self) -> None:
        req: SetParameters.Request = SetParameters.Request()
        req.parameters.append(
            Parameter(
                name="/local_costmap/width",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER, integer_value=4
                ),
            )
        )
        req.parameters.append(
            Parameter(
                name="/local_costmap/height",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER, integer_value=4
                ),
            )
        )
        req.parameters.append(
            Parameter(
                name="/PalLocalPlanner/max_vel_x",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=0.6
                ),
            )
        )
        req.parameters.append(
            Parameter(
                name="/recovery_behavior_enabled",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_BOOL, bool_value=True
                ),
            )
        )
        req.parameters.append(
            Parameter(
                name="/clearing_rotation_allowed",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_BOOL, bool_value=False
                ),
            )
        )
        req.parameters.append(
            Parameter(
                name="/inflation_radius",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=0.2
                ),
            )
        )

        future: rclpy.task.Future = self._set_nav2_parameters.call_async(req)
        rclpy.spin_until_future_complete(self, future)


def main(args: Optional[List[str]] = None):
    rclpy.init(args=args)
    person_following_server_node: PersonFollowingServer = PersonFollowingServer()
    rclpy.spin(person_following_server_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
