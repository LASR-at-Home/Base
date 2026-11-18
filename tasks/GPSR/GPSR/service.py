#!/usr/bin/env python3
"""Callable service for singular GPSR"""

import os
import sys
import string

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration


import yasmin
import yasmin_ros
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from threading import Thread
from rclpy.executors import MultiThreadedExecutor as Executor
from GPSR.states import (
    DispatchSkill,
    ListenState,
    QueryLLM,
)
from GPSR.tts import say

from std_msgs.msg import Empty
from lasr_helper_interfaces.srv import GPSRQuery

from GPSR.planner import PLAN_FAILED_TOKEN
from GPSR.validation import is_yes

from lasr_skills import (
    Say,
    GoToLocation,
    AskAndListen,
    PlayMotion,
    DetectDoorOpening,
    Listen,
    Wait,
)


def _ensure_params_file() -> None:
    if any(a == "--params-file" for a in sys.argv):
        return
    params = os.path.join(get_package_share_directory("GPSR"), "config", "params.yaml")
    sys.argv.extend(["--ros-args", "--params-file", params])


class GPSR_sm(yasmin.StateMachine):
    """Single-shot GPSR flow for the /gpsr/single_query service.

    Runs exactly one ask -> understand -> plan -> readback cycle, then
    either dispatches the resulting skill (dispatch=True) or just
    reports that it can't dispatch right now (dispatch=False, the default).
    Unlike the full multi-command GPSR task in state_machine.py, this does
    not loop to collect further commands or store plans for later.
    """

    def __init__(self, node, dispatch: bool = False, command: str = ""):
        super().__init__(outcomes=["succeeded", "failed"])

        self.node = node
        self.dispatch = dispatch
        # A command passed in the service call replaces the microphone
        self.command = command.strip()
        self.understand_attempts = 0
        self.operator_attempts = 0

        if not self.command:  # no command given: listen on the microphone
            self.add_state(
                "SET_PROMPT",
                yasmin.CbState(outcomes=["succeeded"], callback=self.setPrompt),
                transitions={"succeeded": "REQUEST_AND_WAIT_FOR_COMMAND"},
            )

            self.add_state(
                "REQUEST_AND_WAIT_FOR_COMMAND",
                AskAndListen(),
                transitions={
                    "succeeded": "CHECK_TRANSCRIPT",
                    "failed": "CHECK_TRANSCRIPT",
                },
                remappings={"tts_phrase": "instruction_text"},
            )

            # Speech recognition errors are caught here, before planning: the robot
            # repeats what it heard and only plans it if the operator says yes
            self.add_state(
                "SET_CONFIRM_PROMPT",
                yasmin.CbState(outcomes=["succeeded"], callback=self.setConfirmPrompt),
                transitions={"succeeded": "CONFIRM_COMMAND"},
            )

            self.add_state(
                "CONFIRM_COMMAND",
                AskAndListen(),
                transitions={
                    "succeeded": "CHECK_CONFIRMATION",
                    "failed": "CHECK_CONFIRMATION",
                },
                remappings={
                    "tts_phrase": "confirm_text",
                    "transcribed_speech": "confirmation",
                },
            )

            self.add_state(
                "CHECK_CONFIRMATION",
                yasmin.CbState(
                    outcomes=["yes", "no"], callback=self.checkConfirmation
                ),
                transitions={
                    "yes": "QUERY_LLM",
                    "no": "COUNT_SPEECH_FAILURE",
                },
            )
            after_transcript = "SET_CONFIRM_PROMPT"

        else:
            self.add_state(
                "REQUEST_AND_WAIT_FOR_COMMAND",
                yasmin.CbState(outcomes=["succeeded"], callback=self.useCommand),
                transitions={
                    # Through CHECK_TRANSCRIPT so that `placeholders` is set for
                    # the "Command heard: {}" Say that runs alongside the planner
                    "succeeded": "CHECK_TRANSCRIPT",
                },
            )
            after_transcript = "QUERY_LLM"  # given text needs no confirmation

        self.add_state(
            "CHECK_TRANSCRIPT",
            yasmin.CbState(
                outcomes=["valid", "invalid"],
                callback=self.checkTranscript,
            ),
            transitions={
                "valid": after_transcript,
                "invalid": "COUNT_SPEECH_FAILURE",
            },
        )

        # LLM Query (Planner)
        plan_con = yasmin.Concurrence(
            states={
                "QUERY_LLM": QueryLLM(node),
                "SAY_PLANNING": Say(
                    format_str="Command heard: {}. Give me a moment, I am planning."
                ),
            },
            default_outcome="failed",
            outcome_map={
                "succeeded": {"QUERY_LLM": "succeeded"},
                "failed": {"QUERY_LLM": "failed"},
            },
        )

        self.add_state(
            "QUERY_LLM",
            plan_con,
            transitions={
                "succeeded": "CHECK_OUTCOME",
                "failed": "COUNT_SPEECH_FAILURE",
            },
        )

        self.add_state(
            "CHECK_OUTCOME",
            yasmin.CbState(
                outcomes=[
                    "succeeded",
                    "failed",
                    "request_rephrase",
                    "request_operator",
                    "no_command",
                ],
                callback=self.checkOutcome,
            ),
            transitions={
                "succeeded": "READBACK",
                "request_rephrase": "REQUEST_REPHRASE",
                "request_operator": "REQUEST_OPERATOR",
                "failed": "UNABLE_TO_UNDERSTAND",
                "no_command": "UNABLE_TO_UNDERSTAND",
            },
        )

        # Bad outcome handle
        self.add_state(
            "COUNT_SPEECH_FAILURE",
            yasmin.CbState(
                outcomes=["request_rephrase", "request_operator", "failed"],
                callback=self.countSpeechFailure,
            ),
            transitions={
                "request_rephrase": "REQUEST_REPHRASE",
                "request_operator": "REQUEST_OPERATOR",
                "failed": "UNABLE_TO_UNDERSTAND",
            },
        )

        self.add_state(
            "REQUEST_REPHRASE",
            Say(text="I'm sorry, could you rephrase the command."),
            transitions={
                "succeeded": "REQUEST_AND_WAIT_FOR_COMMAND",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "REQUEST_OPERATOR",
            Say(
                text="I am having trouble understanding. Could the operator please state the command for me?"
            ),
            transitions={
                "succeeded": "REQUEST_AND_WAIT_FOR_COMMAND",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "UNABLE_TO_UNDERSTAND",
            Say(text="I cannot understand or do that currently."),
            transitions={
                "succeeded": "failed",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        readback_state = yasmin.CbState(outcomes=["succeeded"], callback=self.readback)
        readback_state.add_input_key("transcribed_speech")
        readback_state.add_input_key("steps")
        self.add_state(
            "READBACK",
            readback_state,
            transitions={"succeeded": "CHECK_DISPATCH"},
        )

        # Dispatcher
        self.add_state(
            "CHECK_DISPATCH",
            yasmin.CbState(
                outcomes=["dispatch", "no_dispatch"],
                callback=self.checkDispatch,
            ),
            transitions={
                "dispatch": "PRE_NAV",
                "no_dispatch": "SAY_NO_DISPATCH",
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
                "succeeded": "succeeded",
                "failed": "failed",
            },
        )

        self.add_state(
            "SAY_NO_DISPATCH",
            Say(text="I am unable to dispatch skills right now."),
            transitions={
                "succeeded": "succeeded",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

    def useCommand(self, blackboard):
        blackboard["transcribed_speech"] = self.command
        return "succeeded"

    def setPrompt(self, blackboard):
        blackboard["instruction_text"] = "I am ready for your command. Please state it."
        return "succeeded"

    def setConfirmPrompt(self, blackboard):
        blackboard["command_confirmed"] = False
        blackboard["confirm_text"] = (
            f"I heard: {blackboard['placeholders']}. Is that correct? Please say yes or no."
        )
        return "succeeded"

    def checkConfirmation(self, blackboard):
        """'yes' only for a clear yes; anything else (no, unclear, silence) asks again."""
        try:
            answer = str(blackboard["confirmation"])
        except Exception:
            answer = ""
        self.node.get_logger().info(f"Confirmation answer: '{answer}'")
        if is_yes(answer):
            blackboard["command_confirmed"] = True
            return "yes"
        return "no"

    def checkDispatch(self, blackboard):
        return "dispatch" if self.dispatch else "no_dispatch"

    def countSpeechFailure(self, blackboard):
        if self.command:
            return "failed"

        self.understand_attempts += 1

        if self.understand_attempts <= 3:
            return "request_rephrase"

        self.operator_attempts += 1

        if self.operator_attempts <= 3:
            return "request_operator"

        return "failed"

    def checkOutcome(self, blackboard):
        steps = blackboard["steps"]
        #yasmin.YASMIN_LOG_INFO(f"{steps}")

        if len(steps) == 1 and steps[0].get("skill") == "say":
            say_text = (steps[0].get("args") or {}).get("text", "")
            if say_text in ("no command given", "."):
                return "no_command"

            if say_text == PLAN_FAILED_TOKEN:
                if self.command:
                    return "failed"
                self.understand_attempts += 1
                if self.understand_attempts <= 3:
                    return "request_rephrase"
                self.operator_attempts += 1
                if self.operator_attempts <= 3:
                    return "request_operator"
                return "failed"

            # Legitimate say-only response — treat as a normal plan
            return "succeeded"

        self.understand_attempts = 0
        self.operator_attempts = 0

        return "succeeded"

    def readback(self, blackboard):
        try:
            command = blackboard["transcribed_speech"].strip()
            steps = blackboard["steps"]
            announcement = ""
            if steps and steps[0].get("skill") == "say":
                announcement = steps[0]["args"]["text"]
            confirmed = (
                "command_confirmed" in blackboard.keys()
                and blackboard["command_confirmed"]
            )
            if command and not confirmed:  # already read back when confirming
                say(self.node, f"I heard: {command}.", blackboard)
            if announcement and announcement != PLAN_FAILED_TOKEN:
                say(self.node, announcement, blackboard)
        except Exception:
            pass
        return "succeeded"

    def checkTranscript(self, blackboard):
        try:
            text = str(blackboard["transcribed_speech"]).strip()
        except Exception:
            return "invalid"

        if not text:
            return "invalid"

        for ch in "!,.;:?\"'-":
            text = text.replace(ch, "")

        if len(text.split()) < 3:
            return "invalid"

        blackboard["placeholders"] = text
        return "valid"


# GPSR Service node (seperate from State Machine)
class GPSR_service(Node):
    def __init__(self, node):
        super().__init__("gpsr_service")
        self.node = node
        self.srv = self.create_service(GPSRQuery, "/gpsr/single_query", self.callback)

    def callback(self, request: GPSRQuery.Request, response: GPSRQuery.Response):
        command = request.command.data
        self.get_logger().info(
            f"Incoming request: command='{command or '<microphone>'}', "
            f"dispatch_plan={request.dispatch_plan}"
        )

        sm = GPSR_sm(self.node, dispatch=request.dispatch_plan, command=command)
        outcome = sm()

        response.success = outcome == "succeeded"
        response.message = f"GPSR run finished with outcome: {outcome}"
        return response


# GPSR StateMachine node
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


def main():
    _ensure_params_file()
    rclpy.init()
    node = GPSRNode()

    minimal_service = GPSR_service(node)

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
