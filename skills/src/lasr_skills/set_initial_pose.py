from typing import Union
import math
import rclpy

import yasmin
from yasmin import StateMachine, State, Blackboard
import yasmin_ros

from geometry_msgs.msg import Point, Quaternion, Pose, PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
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

        self.node = yasmin_ros.logger_node
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

        initial_pose = PoseWithCovarianceStamped(
            header=Header(frame_id="map", stamp=self.node.get_clock().now().to_msg())
        )
        initial_pose.pose.pose = pose
        initial_pose.pose.covariance = self.covariance

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        publisher = self.node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", qos
        )
        publisher.publish(initial_pose)

        return "succeeded"


def main():
    rclpy.init()

    node = rclpy.create_node(
        "set_initial_pose",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )
    yasmin_ros.set_ros_loggers(node)

    x = node.get_parameter_or("x", None)
    y = node.get_parameter_or("y", None)
    yaw = node.get_parameter_or("yaw", None)
    x = x.value if x is not None else None
    y = y.value if y is not None else None
    yaw = yaw.value if yaw is not None else 0.0

    try:
        sm = StateMachine(outcomes=["succeeded", "failed"])
        if x is not None and y is not None:
            state = SetInitialPose(x=x, y=y, yaw=yaw)
        else:
            state = SetInitialPose(location_param="start_pose")
        sm.add_state(
            "SET_INITIAL_POSE",
            state,
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
