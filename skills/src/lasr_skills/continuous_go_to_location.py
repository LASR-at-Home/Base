from typing import Union
import rclpy
from rclpy.time import Time

import yasmin
from yasmin import StateMachine, State, Blackboard
import yasmin_ros
import time

from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Header



class ContinuousGoToLocation(State):
    """
    Similar to GoToLocation, but is intend for constant navigation, concurrent to another process.
    """
    def __init__(self):
        # This state only exits if the Concurrence kills it or Nav2 crashes
        super().__init__(outcomes=["succeeded", "failed"])
        self.node = yasmin_ros.logger_node
        self.navigator = BasicNavigator()
        self.last_goal = None

    def execute(self, blackboard: Blackboard):
        if "stop_robot_requested" not in blackboard.keys():
            blackboard["stop_robot_requested"] = False
        if "location" not in blackboard.keys():
            blackboard["location"] = None
        while not self.is_canceled() and rclpy.ok():
            stop_requested = blackboard["stop_robot_requested"]
            current_goal = blackboard["location"]

            # Handle Idling or stop requests
            if stop_requested:
                if not self.navigator.isTaskComplete():
                    self.navigator.cancelTask()
                    self.node.get_logger().warn("Safety trigger: Brakes applied. Waiting...")
                time.sleep(0.2)
                continue  

            # Handle missing Goals
            if current_goal is None:
                time.sleep(0.2)
                continue

            # Update Goal
            if self.last_goal is None or self.isMoveableDistance(current_goal, self.last_goal):
                
                self.node.get_logger().info("Nav2: Sending updated goal...")
                goal_stamped = PoseStamped(pose=current_goal, header=Header(frame_id="map", stamp=Time().to_msg()))
                
                self.navigator.goToPose(goal_stamped)
                self.last_goal = current_goal

            # Rest briefly before checking blackboard again
            time.sleep(0.2)

        # Cleanup if the Concurrence cancels this state
        if not self.navigator.isTaskComplete():
            self.navigator.cancelTask()
            
        return "succeeded"

    def isMoveableDistance(self, new_pose: Pose, old_pose: Pose) -> bool:
        dx = new_pose.position.x - old_pose.position.x
        dy = new_pose.position.y - old_pose.position.y
        return (dx**2 + dy**2)**0.5 > 0.25


def main():
    rclpy.init()

    node = rclpy.create_node("hri")
    yasmin_ros.set_ros_loggers(node)

    try:
        sm = StateMachine(outcomes=["succeeded", "failed"])
        sm.add_state(
            "GO_TO_START",
            ContinuousGoToLocation(),
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
