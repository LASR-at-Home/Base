#!/usr/bin/env python3

import rclpy
import yasmin
import yasmin_ros
from rclpy.node import Node
from threading import Thread

try:
    from rclpy.executors import EventsExecutor as Executor
except ImportError:
    from rclpy.executors import MultiThreadedExecutor as Executor

from GPSR.states import AnnouncePlan, DispatchSkill, QueryLLM, create_input_state


def _declare_param_if_needed(node, name, default):
    if not node.has_parameter(name):
        node.declare_parameter(name, default)


class GPSRNode(Node):
    def __init__(self):
        super().__init__(
            node_name="gpsr",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self._executor = Executor()
        self._executor.add_node(self)
        self._spin_thread = Thread(target=self._executor.spin)
        self._spin_thread.start()


def main(args=None):
    rclpy.init(args=args)
    node = GPSRNode()

    _declare_param_if_needed(node, "llm_model", "llama3.2")
    _declare_param_if_needed(node, "llm_host", "http://localhost:11434")
    _declare_param_if_needed(node, "locations_package", "GPSR")
    _declare_param_if_needed(node, "locations_file", "config/locations.yaml")
    _declare_param_if_needed(node, "simulation", False)
    _declare_param_if_needed(node, "input_mode", "keyboard")
    _declare_param_if_needed(node, "input_prompt", "Enter command: ")

    input_mode = node.get_parameter("input_mode").value
    simulation = node.get_parameter("simulation").value
    node.get_logger().info(
        f"Starting GPSR state machine (input_mode={input_mode}, simulation={simulation})..."
    )

    yasmin_ros.set_ros_loggers(node)

    sm = yasmin.StateMachine(outcomes=["succeeded", "failed"], handle_sigint=True)

    sm.add_state(
        "WAIT_FOR_COMMAND",
        create_input_state(node),
        transitions={
            "succeeded": "QUERY_LLM",
            "aborted": "WAIT_FOR_COMMAND",
        },
    )
    sm.add_state(
        "QUERY_LLM",
        QueryLLM(node),
        transitions={
            "succeeded": "ANNOUNCE_PLAN",
            "failed": "WAIT_FOR_COMMAND",
        },
    )
    sm.add_state(
        "ANNOUNCE_PLAN",
        AnnouncePlan(node),
        transitions={
            "succeeded": "DISPATCH_SKILL",
        },
    )
    sm.add_state(
        "DISPATCH_SKILL",
        DispatchSkill(node),
        transitions={
            "succeeded": "WAIT_FOR_COMMAND",
            "failed": "WAIT_FOR_COMMAND",
        },
    )

    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(f"State machine finished with outcome: {outcome}")
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(str(e))
    finally:
        node._executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
