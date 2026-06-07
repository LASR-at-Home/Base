import yasmin
import yasmin_ros
import rclpy
from control_msgs.action import PointHead
from geometry_msgs.msg import Point, PointStamped, Vector3
from std_msgs.msg import Header

from rclpy.callback_groups import ReentrantCallbackGroup

from typing import Union

ros_client_group = ReentrantCallbackGroup()


class LookToPoint(yasmin_ros.ActionState):
    def __init__(
        self,
        pointstamped: Union[None, PointStamped] = None,
    ):

        super().__init__(
            action_name="/head_controller/point_head_action",
            action_type=PointHead,
            create_goal_handler=self._create_goal,
            response_timeout=5.0,
            callback_group=ros_client_group,
            maximum_retry=1,
        )
        if pointstamped is None:
            self.add_input_key("pointstamped")
        self._pointstamped = pointstamped

    def _create_goal(self, blackboard):
        target = (
            self._pointstamped
            if self._pointstamped is not None
            else blackboard["pointstamped"]
        )

        goal = PointHead.Goal()
        goal.pointing_frame = "head_front_camera_depth_optical_frame"
        goal.pointing_axis = Vector3(x=0.0, y=0.0, z=1.0)
        goal.max_velocity = 1.0
        goal.target = target

        yasmin.YASMIN_LOG_INFO(
            "Sending PointHead goal: "
            f"frame={goal.target.header.frame_id}, "
            f"point=({goal.target.point.x:.3f}, "
            f"{goal.target.point.y:.3f}, {goal.target.point.z:.3f})"
        )

        return goal


def main():
    rclpy.init()

    yasmin_ros.set_ros_loggers()

    sm = yasmin.StateMachine(outcomes=["succeeded", "failed"], handle_sigint=True)

    sm.add_state(
        "LOOK",
        LookToPoint(
            pointstamped=PointStamped(
                header=Header(frame_id="base_link"),
                point=Point(x=2.170, y=0.536, z=0.700),
            )
        ),
        transitions={
            "succeeded": "succeeded",
            "aborted": "failed",
            "canceled": "failed",
        },
    )

    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(f"State machine finished with outcome {outcome}")
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)

    if rclpy.ok():
        rclpy.shutdown()
