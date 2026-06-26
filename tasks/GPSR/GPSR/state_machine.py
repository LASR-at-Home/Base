#!/usr/bin/env python3

import os
import sys

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

from std_msgs.msg import Empty

from lasr_skills import (
    Say,
    GoToLocation,
    StopEyeTracker,
    PlayMotion,
    StartDoorSM,
    Listen,
)


def _ensure_params_file() -> None:
    if any(a == "--params-file" for a in sys.argv):
        return
    params = os.path.join(get_package_share_directory("GPSR"), "config", "params.yaml")
    sys.argv.extend(["--ros-args", "--params-file", params])


class GPSR(yasmin.StateMachine):
    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "failed"])

        self.instruction_count = 1
        self.understand_attempts = 0

        # Wait for Start signal at the door
        self.add_state(
            "WAIT_START",  # Awaits start Signal for the task
            yasmin_ros.MonitorState(
                topic_name="/gpsr/start",
                outcomes=["succeeded", "failed"],
                monitor_handler=self.wait_cb,
                msg_type=Empty,
            ),
            transitions={
                "succeeded": "START_CON",
                "failed": "WAIT_START",
                "canceled": "failed",
            },
        )

        self.add_state(
            "START_CON",  # SM1: Waits for Door to open, then goes to start
            self.setup(),
            transitions={"succeeded": "GO_TO_INSTRUCT_POINT", "failed": "START_CON"},
        )

        # Main Task loop:
        self.add_state(
            "GO_TO_INSTRUCT_POINT",
            GoToLocation(location_param="instruction_point"),
            transitions={"succeeded": "POST_NAV", "failed": "failed"},
        )

        self.add_state(
            "POST_NAV",
            PlayMotion("post_navigation"),
            transitions={
                "succeeded": "CHECK_INSTRUCTION",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "CHECK_INSTRUCTION",
            yasmin.CbState(outcomes=["next", "finish"], callback=self.checkRequest),
            transitions={"next": "REQUEST_INTRUCTION", "finish": "succeeded"},
        )

        self.add_state(  # Swap for ask and listen? then check then count number of requests/ if want to exit
            "REQUEST_INTRUCTION",
            Say(),
            transitions={
                "succeeded": "WAIT_FOR_COMMAND",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "WAIT_FOR_COMMAND",
            Listen(),
            transitions={
                "succeeded": "QUERY_LLM",
                "aborted": "WAIT_FOR_COMMAND",
            },
        )
        self.add_state(
            "QUERY_LLM",
            QueryLLM(node),
            transitions={
                "succeeded": "PRE_NAV",
                "failed": "WAIT_FOR_COMMAND",
            },
        )

        self.add_state(
            "PRE_NAV",
            PlayMotion("pre_navigation"),
            transitions={
                "succeeded": "DISPATCH_SKILL",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "DISPATCH_SKILL",
            DispatchSkill(node),
            transitions={
                "succeeded": "CHECK_OUTCOME",
                "failed": "CHECK_OUTCOME",
            },
        )
        self.add_state(
            "CHECK_OUTCOME",
            yasmin.CbState(
                outcomes=["succeeded", "failed", "no_command"],
                callback=self.checkOutcome,
            ),
            transitions={
                "succeeded": "SAY_COMPLETE",
                "failed": "UNABLE_TO_UNDERSTAND",
                "no_command": "GO_TO_INSTRUCT_POINT",
            },
        )

        self.add_state(
            "SAY_COMPLETE",
            Say(
                text="I have finished doing the task. I will go back to the instruction point."
            ),
            transitions={
                "succeeded": "GO_TO_INSTRUCT_POINT",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "UNABLE_TO_UNDERSTAND",
            Say(text="I cannot understand that task."),
            transitions={
                "succeeded": "GO_TO_INSTRUCT_POINT",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

    def wait_cb(self, blackboard, msg):
        yasmin.YASMIN_LOG_INFO("RECEIVED START SIGNAL")
        return "succeeded"

    def setup(self):
        start_con_sm = yasmin.Concurrence(
            states={
                "SAY_START": Say(text="Start of G P S R task."),
                "DOOR_START": StartDoorSM(location_param="instruction_point"),
            },
            default_outcome="failed",
            outcome_map={
                "succeeded": {
                    "SAY_START": "succeeded",
                    "DOOR_START": "succeeded",
                },
                "failed": {
                    "SAY_START": "aborted",
                    "DOOR_START": "failed",
                },
            },
        )

        return start_con_sm

    def checkRequest(self, blackboard):
        if self.instruction_count == 1:
            blackboard["text"] = "I am ready for the first command. "
        elif self.instruction_count == 2:
            blackboard["text"] = "I am ready for the second command. "
        elif self.instruction_count == 3:
            blackboard["text"] = "I am ready for the last command. "
        else:
            return "finish"

        return "next"

        # Callback to check the blackboard["steps"]
        #  - if it is only 1 say and the say is one of the fails then dont increment intructs but fail count
        #  - if say but a "no command given" ignore
        #  - if full plan: reset fails and increment intruction

        # TODO: Check if plan is possible/ conductable - Skill may be to just say something (like time or description)
        # if len(steps) == 1 and steps[0].get("skill") == "say":
        #     if (plan["plan_description"] == "I could not generate a plan for that command." or
        #         plan["plan_description"] == "I'm sorry, I don't know how to do that."):

    def checkOutcome(self, blackboard):
        steps = blackboard["steps"]
        yasmin.YASMIN_LOG_INFO(f"{steps}")

        if len(steps) == 1 and steps[0].get("skill") == "say":
            say_text = steps[0].get("args").get("text")
            if say_text == "no command given" or say_text == ".":
                return "no_outcome"

            # If there is only 1 skill then assume it wasndidnt understand and ask for a rephrase/ repeat
            if (
                say_text == "I could not generate a plan for that command."
                or say_text == "I'm sorry, I don't know how to do that."
            ):
                self.understand_attempts += 1

                if self.understand_attempts > 3:
                    return "failed"

                return "success"  # Don't increment

        self.instruction_count += 1
        self.understand_attempts = 0

        return "succeeded"


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

    # if input_mode == "keyboard":
    #     wait_for_command = KeyboardInputState(node)
    # elif input_mode in ("mic", "microphone"):
    #     wait_for_command = ListenState()

    # yasmin_ros.set_ros_loggers(node)

    # sm = yasmin.StateMachine(outcomes=["succeeded", "failed"], handle_sigint=True)

    # sm.add_state(
    #     "WAIT_FOR_COMMAND",
    #     wait_for_command,
    #     transitions={
    #         "succeeded": "QUERY_LLM",
    #         "aborted": "WAIT_FOR_COMMAND",
    #     },
    # )
    # sm.add_state(
    #     "QUERY_LLM",
    #     QueryLLM(node),
    #     transitions={
    #         "succeeded": "DISPATCH_SKILL",
    #         "failed": "WAIT_FOR_COMMAND",
    #     },
    # )
    # sm.add_state(
    #     "DISPATCH_SKILL",
    #     DispatchSkill(node),
    #     transitions={
    #         "succeeded": "WAIT_FOR_COMMAND",
    #         "failed": "WAIT_FOR_COMMAND",
    #     },
    # )

    yasmin_ros.set_ros_loggers(node)

    sm = GPSR(node)
    sm.set_sigint_handler(True)

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
