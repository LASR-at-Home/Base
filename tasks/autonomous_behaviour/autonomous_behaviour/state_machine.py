#!/usr/bin/env python3
import sys
import threading
from pathlib import Path

import rclpy
import smach
from rclpy.executors import MultiThreadedExecutor

sys.path.insert(0, str(Path(__file__).parents[3] / "skills" / "src"))

from autonomous_behaviour.states import DispatchSkill, ListenState, QueryLLM


class AutonomousBehaviour(smach.StateMachine):
    """Top-level state machine: listen → query LLM → dispatch skill → repeat."""

    def __init__(self, node):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:
            smach.StateMachine.add(
                "WAIT_FOR_COMMAND",
                ListenState(node),
                transitions={
                    "succeeded": "QUERY_LLM",
                    "aborted": "WAIT_FOR_COMMAND",
                    "preempted": "WAIT_FOR_COMMAND",
                },
            )
            smach.StateMachine.add(
                "QUERY_LLM",
                QueryLLM(node),
                transitions={
                    "succeeded": "DISPATCH_SKILL",
                    "failed": "WAIT_FOR_COMMAND",
                },
            )
            smach.StateMachine.add(
                "DISPATCH_SKILL",
                DispatchSkill(node),
                transitions={
                    "succeeded": "WAIT_FOR_COMMAND",
                    "failed": "WAIT_FOR_COMMAND",
                },
            )


class AutonomousBehaviourNode(rclpy.node.Node):
    """ROS2 node that owns the state machine and spins it on a multi-threaded executor.

    The MultiThreadedExecutor is required because nested calls (e.g. Nav2's
    BasicNavigator) run their own spin_until_future_complete; a single-threaded
    executor would deadlock.
    """

    def __init__(self):
        super().__init__("autonomous_behaviour_node")
        self.sm = AutonomousBehaviour(self)

    def run(self):
        """Start the SM in a background thread and spin ROS callbacks in the foreground."""
        self.get_logger().info("Starting state machine...")
        executor = MultiThreadedExecutor()
        executor.add_node(self)
        sm_thread = threading.Thread(target=self.sm.execute)
        sm_thread.start()
        try:
            executor.spin()
        finally:
            sm_thread.join()


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousBehaviourNode()
    node.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
