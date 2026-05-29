#!/usr/bin/env python3
import threading

import rclpy
import smach
from rclpy.executors import MultiThreadedExecutor

from GPSR.states import DispatchSkill, QueryLLM, create_input_state


class GPSR(smach.StateMachine):
    """Top-level state machine: listen → query LLM → dispatch skill → repeat."""

    def __init__(self, node):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:
            smach.StateMachine.add(
                "WAIT_FOR_COMMAND",
                create_input_state(node),
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


def _declare_param_if_needed(node, name, default):
    if not node.has_parameter(name):
        node.declare_parameter(name, default)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node(
        node_name="gpsr",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )
    _declare_param_if_needed(node, "llm_model", "llama3.2")
    _declare_param_if_needed(node, "llm_host", "http://localhost:11434")
    _declare_param_if_needed(node, "locations_package", "GPSR")
    _declare_param_if_needed(node, "locations_file", "config/locations.yaml")
    _declare_param_if_needed(node, "input_mode", "keyboard")
    _declare_param_if_needed(node, "input_prompt", "Enter command: ")

    input_mode = node.get_parameter("input_mode").value
    node.get_logger().info(f"Starting GPSR state machine (input_mode={input_mode})...")
    sm = GPSR(node)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    sm_thread = threading.Thread(target=sm.execute)
    sm_thread.start()
    try:
        executor.spin()
    finally:
        sm_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
