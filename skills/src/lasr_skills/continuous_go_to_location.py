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
        super().__init__(outcomes=["succeeded", "canceled", "aborted"])
        self.node = yasmin_ros.logger_node
        self.navigator = BasicNavigator()
        self.last_goal = None

        self.add_input_key("location")
        self.add_input_key("cancel_nav")
        self.add_input_key("stop_robot_requested")

        self.add_output_key("stop_robot_requested")

    def execute(self, blackboard: Blackboard):
        if "stop_robot_requested" not in blackboard.keys():
            blackboard["stop_robot_requested"] = False
        if "location" not in blackboard.keys():
            blackboard["location"] = None
        if "cancel_nav" not in blackboard.keys():
            blackboard["cancel_nav"] = False

        while not self.is_canceled() and rclpy.ok():
            current_goal = blackboard["location"]

            if blackboard["cancel_nav"]:
                blackboard["stop_robot_requested"] = True
                self.cancel_state()

            # Handle Idling or stop requests
            if blackboard["stop_robot_requested"]:
                if not self.navigator.isTaskComplete():
                    self.navigator.cancelTask()
                    self.node.get_logger().warn("Stop Requested")
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

            # Kill early as long as well are close enough to the goal.
            if not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback and hasattr(feedback, 'distance_remaining'):
                    if feedback.distance_remaining <= 0.15:
                        self.node.get_logger().info(
                            f"Nav2: Roughly at goal ({feedback.distance_remaining:.2f}m). Canceling exact approach."
                        )
                        self.navigator.cancelTask()
                        # Tell the state machine we are idling now so it doesn't immediately resend
                        blackboard["stop_robot_requested"] = True
                        self.last_goal = None

            # Rest briefly before checking blackboard again
            time.sleep(0.2)

        # Cleanup if the Concurrence cancels this state
        if not self.navigator.isTaskComplete():
            self.navigator.cancelTask()
        
        if self.is_canceled():
            return "canceled"

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
