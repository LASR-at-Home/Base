#!/usr/bin/env python3

import os
import sys

import rclpy
import yasmin
import yasmin_ros
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from threading import Thread
from rclpy.executors import MultiThreadedExecutor as Executor
from GPSR.states import DispatchSkill, KeyboardInputState, ListenState, QueryLLM


def _ensure_params_file() -> None:
    if any(a == "--params-file" for a in sys.argv):
        return
    params = os.path.join(get_package_share_directory("GPSR"), "config", "params.yaml")
    sys.argv.extend(["--ros-args", "--params-file", params])


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
    _ensure_params_file()
    rclpy.init(args=args)
    node = GPSRNode()

    input_mode = node.get_parameter("input_mode").value.strip().lower()
    simulation = node.get_parameter("simulation").value
    node.get_logger().info(
        f"Starting GPSR state machine (input_mode={input_mode}, simulation={simulation})..."
    )

    if input_mode == "keyboard":
        wait_for_command = KeyboardInputState(node)
    elif input_mode in ("mic", "microphone"):
        wait_for_command = ListenState(node)

    yasmin_ros.set_ros_loggers(node)

    sm = yasmin.StateMachine(outcomes=["succeeded", "failed"], handle_sigint=True)

    sm.add_state(
        "WAIT_FOR_COMMAND",
        wait_for_command,
        transitions={
            "succeeded": "QUERY_LLM",
            "aborted": "WAIT_FOR_COMMAND",
        },
    )
    sm.add_state(
        "QUERY_LLM",
        QueryLLM(node),
        transitions={
            "succeeded": "DISPATCH_SKILL",
            "failed": "WAIT_FOR_COMMAND",
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
