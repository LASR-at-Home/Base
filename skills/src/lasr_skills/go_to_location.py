from typing import Union
import rclpy
from smach_ros import RosState
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Header

# INFO: individual file can be ran with .yamlusing command: --ros-args --params-file {path}.yaml


class GoToLocation(RosState):
    def __init__(
        self,
        node,
        location: Union[Pose, None] = None,
        location_param: Union[str, None] = None,
    ):

        if location is not None or location_param is not None:
            super().__init__(node, outcomes=["succeeded", "failed"])
        else:
            super().__init__(
                node, outcomes=["succeeded", "failed"], input_keys=["location"]
            )

        self.navigator = BasicNavigator()
        self.location = location
        self.location_param = (
            location_param  # the pose (eg. 'start_pose', 'wait_pose', ...)
        )

    def execute(self, userdata):
        if self.location:
            goal_pose = self.location
        elif self.location_param:
            goal_pose = Pose(
                position=Point(
                    x=float(
                        self.node.get_parameter(
                            f"{self.location_param}.position.x"
                        ).value
                    ),
                    y=float(
                        self.node.get_parameter(
                            f"{self.location_param}.position.y"
                        ).value
                    ),
                    z=float(
                        self.node.get_parameter(
                            f"{self.location_param}.position.z"
                        ).value
                    ),
                ),
                orientation=Quaternion(
                    x=float(
                        self.node.get_parameter(
                            f"{self.location_param}.orientation.x"
                        ).value
                    ),
                    y=float(
                        self.node.get_parameter(
                            f"{self.location_param}.orientation.y"
                        ).value
                    ),
                    z=float(
                        self.node.get_parameter(
                            f"{self.location_param}.orientation.z"
                        ).value
                    ),
                    w=float(
                        self.node.get_parameter(
                            f"{self.location_param}.orientation.w"
                        ).value
                    ),
                ),
            )

        elif "location" in userdata:
            goal_pose = userdata.location
        else:
            return "failed"

        goal_stamped = PoseStamped(pose=goal_pose, header=Header(frame_id="map"))

        self.navigator.goToPose(goal_stamped)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.navigator)

        return (
            "succeeded"
            if self.navigator.getResult() == TaskResult.SUCCEEDED
            else "failed"
        )


def main():
    rclpy.init()

    # Update Node name if loading from a .yaml config
    node = rclpy.create_node(
        "go_to_location",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )

    try:
        state = GoToLocation(node=node, location_param="start_pose")
        outcome = state.execute(userdata={})
        node.get_logger().info(f"GoToLocation outcome: {outcome}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
