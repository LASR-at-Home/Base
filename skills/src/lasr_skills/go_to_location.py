from typing import Union
import rclpy

import yasmin
from yasmin import StateMachine, State, Blackboard
import yasmin_ros
import time

from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Header

# INFO: individual file can be ran with .yaml using command: --ros-args --params-file {path}.yaml


class GoToLocation(State):
    def __init__(
        self,
        location: Union[Pose, None] = None,
        location_param: Union[str, None] = None,
        updatable: bool = False,
    ):

        super().__init__(outcomes=["succeeded", "failed"])
        if not (location is not None or location_param is not None):
            self.add_input_key("location")
        self.node = yasmin_ros.logger_node

        self.navigator = BasicNavigator()
        self.location = location
        self.location_param = location_param  # the pose (eg. 'start_pose', 'wait_pose',
        self.updatable = updatable

    def execute(self, blackboard):
        if self.location:
            goal_pose = self.location
        elif self.location_param:

            goal_pose = Pose(
                position=Point(
                    x=float(
                        self.node.get_parameter(f"{self.location_param}.position.x").value
                    ),
                    y=float(
                        self.node.get_parameter(f"{self.location_param}.position.y").value
                    ),
                    z=float(
                        self.node.get_parameter(f"{self.location_param}.position.z").value
                    ),
                ),
                orientation=Quaternion(
                    x=float(
                        self.node.get_parameter(f"{self.location_param}.orientation.x").value
                    ),
                    y=float(
                        self.node.get_parameter(f"{self.location_param}.orientation.y").value
                    ),
                    z=float(
                        self.node.get_parameter(f"{self.location_param}.orientation.z").value
                    ),
                    w=float(
                        self.node.get_parameter(f"{self.location_param}.orientation.w").value
                    ),
                ),
            )

        elif "location" in blackboard.keys():
            goal_pose = blackboard["location"]
        else:
            return "failed"

        goal_stamped = PoseStamped(pose=goal_pose, header=Header(frame_id="map"))
        self.navigator.goToPose(goal_stamped)

        while (
            not self.navigator.isTaskComplete() and rclpy.ok()
        ):  # Update to make it check if the goal has been updated? (blackboard["location"] is different)
            time.sleep(0.2)

            if "stop_robot_requested" in blackboard.keys() and blackboard["stop_robot_requested"]:
                self.navigator.cancelTask()
                self.node.get_logger().warn("Safety trigger: Person too close! Canceling Nav2 Task.")
                return "failed"
            
            if self.is_canceled():
                if not self.navigator.isTaskComplete():
                    self.navigator.cancelTask()

                return "failed"
            
            if not self.updatable:
                continue

            if (
                not (self.location or self.location_param)
                and "location" in blackboard.keys()
            ):
                new_goal = blackboard["location"]

                if new_goal:
                    # Calculate physical distance between the current goal and the new goal
                    dx = new_goal.position.x - goal_pose.position.x
                    dy = new_goal.position.y - goal_pose.position.y
                    distance_moved = (dx**2 + dy**2)**0.5

                    # Only preempt Nav2 if the person has moved more than 0.25 meters
                    if distance_moved > 0.25:
                        yasmin.YASMIN_LOG_INFO(f"Updated new goal {distance_moved}m away. ")
                        goal_pose = new_goal
                        goal_stamped = PoseStamped(
                            pose=goal_pose, header=Header(frame_id="map")
                        )
                        self.navigator.goToPose(goal_stamped)

        if not rclpy.ok() or self.is_canceled():
            if not self.navigator.isTaskComplete():
                self.navigator.cancelTask()
                time.sleep(0.2)
            return "failed"

        if self.is_canceled():
            return "failed"
        return (
            "succeeded"
            if self.navigator.getResult() == TaskResult.SUCCEEDED
            else "failed"
        )


def main():
    rclpy.init()

    # Update Node name if loading from a .yaml config
    node = rclpy.create_node("hri")
    yasmin_ros.set_ros_loggers(node)

    try:
        sm = StateMachine(outcomes=["succeeded", "failed"])
        sm.add_state(
            "GO_TO_START",
            GoToLocation(location_param="start_pose"),
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
