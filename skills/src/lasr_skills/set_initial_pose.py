from typing import Union
import math
import rclpy

import yasmin
from yasmin import StateMachine, State, Blackboard
import yasmin_ros

from geometry_msgs.msg import Point, Quaternion, Pose, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from std_msgs.msg import Header

# INFO: individual file can be ran with .yaml using command: --ros-args --params-file {path}.yaml

# Small covariance to use when the pose is known accurately (e.g. robot was
# placed at a known, marked location). Diagonal order matches
# PoseWithCovarianceStamped.pose.covariance: x, y, z, roll, pitch, yaw.
SMALL_COVARIANCE = [
    0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787,
]


class SetInitialPose(State):
    def __init__(
        self,
        location: Union[Pose, None] = None,
        location_param: Union[str, None] = None,
        x: Union[float, None] = None,
        y: Union[float, None] = None,
        z: float = 0.0,
        yaw: float = 0.0,
        covariance: Union[list, None] = None,
    ):

        super().__init__(outcomes=["succeeded", "failed"])
        if not (
            location is not None
            or location_param is not None
            or (x is not None and y is not None)
        ):
            self.add_input_key("location")

        self.navigator = BasicNavigator()
        self.location = location
        self.location_param = (
            location_param  # the pose (eg. 'start_pose', 'wait_pose', ...)
        )
        self.xyz = (x, y, z) if x is not None and y is not None else None
        self.yaw = yaw
        self.covariance = covariance if covariance is not None else SMALL_COVARIANCE

    def execute(self, blackboard):
        if self.location:
            pose = self.location
        elif self.xyz:
            x, y, z = self.xyz
            pose = Pose(
                position=Point(x=float(x), y=float(y), z=float(z)),
                orientation=Quaternion(
                    x=0.0,
                    y=0.0,
                    z=math.sin(self.yaw / 2),
                    w=math.cos(self.yaw / 2),
                ),
            )
        elif self.location_param:

            node = yasmin_ros.logger_node

            pose = Pose(
                position=Point(
                    x=float(
                        node.get_parameter(f"{self.location_param}.position.x").value
                    ),
                    y=float(
                        node.get_parameter(f"{self.location_param}.position.y").value
                    ),
                    z=float(
                        node.get_parameter(f"{self.location_param}.position.z").value
                    ),
                ),
                orientation=Quaternion(
                    x=float(
                        node.get_parameter(f"{self.location_param}.orientation.x").value
                    ),
                    y=float(
                        node.get_parameter(f"{self.location_param}.orientation.y").value
                    ),
                    z=float(
                        node.get_parameter(f"{self.location_param}.orientation.z").value
                    ),
                    w=float(
                        node.get_parameter(f"{self.location_param}.orientation.w").value
                    ),
                ),
            )
        elif "location" in blackboard.keys():
            pose = blackboard["location"]
        else:
            return "failed"

        initial_pose = PoseWithCovarianceStamped(header=Header(frame_id="map"))
        initial_pose.pose.pose = pose
        initial_pose.pose.covariance = self.covariance

        self.navigator.setInitialPose(initial_pose)

        return "succeeded"


def main():
    rclpy.init()

    # Update Node name if loading from a .yaml config
    node = rclpy.create_node("hri")
    yasmin_ros.set_ros_loggers(node)

    try:
        sm = StateMachine(outcomes=["succeeded", "failed"])
        sm.add_state(
            "SET_INITIAL_POSE",
            SetInitialPose(location_param="start_pose"),
            transitions={"succeeded": "succeeded", "failed": "failed"},
        )

        bb = Blackboard()
        outcome = sm(bb)

        yasmin.YASMIN_LOG_INFO(outcome)
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
