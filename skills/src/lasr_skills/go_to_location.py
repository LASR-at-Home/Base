from typing import Union
import rclpy

import yasmin
from yasmin import StateMachine, State, Blackboard
import yasmin_ros


from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Header

# INFO: individual file can be ran with .yaml using command: --ros-args --params-file {path}.yaml


class GoToLocation(State):
    def __init__(
        self,
        location: Union[Pose, None] = None,
        location_param: Union[str, None] = None,
    ):

        super().__init__(outcomes=["succeeded", "failed"])
        if not (location is not None or location_param is not None):
            self.add_input_key("location")

        self.navigator = BasicNavigator()
        self.location = location
        self.location_param = (
            location_param  # the pose (eg. 'start_pose', 'wait_pose', 
        )

    def execute(self, blackboard):
        if self.location:
            goal_pose = self.location
        elif self.location_param:

            node = yasmin_ros.logger_node

            goal_pose = Pose(
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
            goal_pose = blackboard["location"]
        else:
            return "failed"

        goal_stamped = PoseStamped(pose=goal_pose, header=Header(frame_id="map"))
        self.navigator.goToPose(goal_stamped)

        rate = node.create_rate(5.0)
        while not self.navigator.isTaskComplete():  # Update to make it check if the goal has been updated? (blackboard["location"] is different)
            if self.is_canceled():
                if not self.navigator.isTaskComplete():
                    self.navigator.cancelTask()

                return "failed"
            
            if not (self.location or self.location_param) and "location" in blackboard.keys():
                new_goal = blackboard["location"]
                
                if new_goal and new_goal != goal_pose:
                    goal_pose = new_goal
                    goal_stamped = PoseStamped(pose=goal_pose, header=Header(frame_id="map"))
                    self.navigator.goToPose(goal_stamped)

            rate.sleep()

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
