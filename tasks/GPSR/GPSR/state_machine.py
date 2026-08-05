#!/usr/bin/env python3

import os
import sys
import string

import rclpy
import yasmin
import yasmin_ros
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from threading import Thread
from rclpy.executors import MultiThreadedExecutor as Executor
from GPSR.states import (
    DispatchSkill,
    KeyboardInputState,
    ListenState,
    QueryLLM,
)
from GPSR.tts import say

from std_msgs.msg import Empty

from GPSR.planner import PLAN_FAILED_TOKEN

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


class GPSR(yasmin.StateMachine):
    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "failed"])

        self.node = node
        self.instruction_count = 1
        self.understand_attempts = 0
        self.operator_attempts = 0

        # Collected plans (one steps-list per accepted command) and
        # the index of the plan currently being executed.
        self.plans = []
        self.exec_index = 0

        self.add_state(
            "WAIT_START",
            yasmin_ros.MonitorState(
                topic_name="/gpsr/start",
                outcomes=["succeeded", "failed"],
                monitor_handler=self.start_cb,
                msg_type=Empty,
            ),
            transitions={
                "succeeded": "START_CON",
                "failed": "WAIT_START",
                "canceled": "failed",
            },
        )

        self.add_state(
            "START_CON",
            self.setup(),
            transitions={"succeeded": "WAIT_3", "failed": "START_CON"},
        )
        self.add_state(
            "WAIT_3",
            Wait(3),
            transitions={
                "succeeded": "ENTER_DOOR",
                "failed": "ENTER_DOOR",
            },
        )

        self.add_state(
            "ENTER_DOOR",
            GoToLocation(location_param="entrance_point"),
            transitions={"succeeded": "GO_TO_INSTRUCT_POINT", "failed": "ENTER_DOOR"},
        )

        self.add_state(
            "GO_TO_INSTRUCT_POINT",
            GoToLocation(location_param="instruction_point"),
            transitions={
                "succeeded": "CHECK_INSTRUCTION",
                "failed": "GO_TO_INSTRUCT_POINT",
            },
        )

        self.add_state(
            "CHECK_INSTRUCTION",
            yasmin.CbState(outcomes=["next", "finish"], callback=self.checkRequest),
            transitions={"next": "REQUEST_AND_WAIT_FOR_COMMAND", "finish": "NEXT_PLAN"},
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

        self.add_state(
            "CHECK_TRANSCRIPT",
            yasmin.CbState(
                outcomes=["valid", "invalid"],
                callback=self.checkTranscript,
            ),
            transitions={
                "valid": "QUERY_LLM",
                "invalid": "COUNT_SPEECH_FAILURE",
            },
        )

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
                "succeeded": "SAY_COMPLETE",
                "failed": "SAY_COMPLETE",
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
                "no_command": "CHECK_INSTRUCTION",
            },
        )

        readback_state = yasmin.CbState(outcomes=["succeeded"], callback=self.readback)
        readback_state.add_input_key("transcribed_speech")
        readback_state.add_input_key("steps")
        self.add_state(
            "READBACK",
            readback_state,
            transitions={"succeeded": "WAIT_BEFORE_NEXT"},
        )

        self.add_state(
            "WAIT_BEFORE_NEXT",
            Wait(5),
            transitions={"succeeded": "STORE_PLAN", "failed": "STORE_PLAN"},
        )

        # Store the accepted plan; collect the next command until we have
        # gathered all requested instructions, then move on to execution.
        self.add_state(
            "STORE_PLAN",
            yasmin.CbState(
                outcomes=["collect_next", "execute"],
                callback=self.storePlan,
            ),
            transitions={
                "collect_next": "CHECK_INSTRUCTION",
                "execute": "NEXT_PLAN",
            },
        )

        # # Announce all collected plans before starting execution.
        # self.add_state(
        #     "ANNOUNCE_ALL",
        #     yasmin.CbState(
        #         outcomes=["succeeded"],
        #         callback=self.announceAll,
        #     ),
        #     transitions={"succeeded": "NEXT_PLAN"},
        # )

        # Execution phase: load one stored plan at a time into the blackboard
        # and run it via PRE_NAV -> DISPATCH_SKILL.
        self.add_state(
            "NEXT_PLAN",
            yasmin.CbState(
                outcomes=["execute", "finish"],
                callback=self.nextPlan,
            ),
            transitions={
                "execute": "PRE_NAV",
                "finish": "SAY_TASK_OVER",
            },
        )

        self.add_state(
            "SAY_COMPLETE",
            Say(
                text="I have finished doing the task. I will go back to the instruction point."
            ),
            transitions={
                "succeeded": "RETURN_TO_INSTRUCT_POINT",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        # Return to the instruction point between executed plans, then load
        # the next stored plan.
        self.add_state(
            "RETURN_TO_INSTRUCT_POINT",
            GoToLocation(location_param="instruction_point"),
            transitions={"succeeded": "NEXT_PLAN", "failed": "NEXT_PLAN"},
        )
        self.add_state(
            "REQUEST_REPHRASE",
            Say(text="Im sorry could you rephrase the command."),
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
            Say(
                text="I cannot understand or do that currently. Please move on to the next command. "
            ),
            transitions={
                "succeeded": "CHECK_INSTRUCTION",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "SAY_TASK_OVER",
            Say(text="I have finished all the tasks. "),
            transitions={
                "succeeded": "succeeded",
                "aborted": "succeeded",
                "canceled": "succeeded",
            },
        )

    def start_cb(self, blackboard, msg):
        yasmin.YASMIN_LOG_INFO("RECEIVED START SIGNAL")
        return "succeeded"

    def setup(self):
        start_con_sm = yasmin.Concurrence(
            states={
                "SAY_START": Say(text="Start of G P S R task."),
                "DOOR_START": DetectDoorOpening(),
            },
            default_outcome="failed",
            outcome_map={
                "succeeded": {
                    "SAY_START": "succeeded",
                    "DOOR_START": "door_opened",
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
            blackboard["instruction_text"] = (
                "I am ready for the first command. Please state it."
            )
        elif self.instruction_count == 2:
            blackboard["instruction_text"] = (
                "I am ready for the second command. Please state it."
            )
        elif self.instruction_count == 3:
            blackboard["instruction_text"] = (
                "I am ready for the last command. Please state it."
            )
        else:
            return "finish"

        yasmin.YASMIN_LOG_INFO(f"{blackboard['instruction_text']}")

        return "next"

        # Callback to check the blackboard["steps"]
        #  - if it is only 1 say and the say is one of the fails then dont increment intructs but fail count
        #  - if say but a "no command given" ignore
        #  - if full plan: reset fails and increment intruction

        # TODO: Check if plan is possible/ conductable - Skill may be to just say something (like time or description)
        # if len(steps) == 1 and steps[0].get("skill") == "say":
        #     if (plan["plan_description"] == "I could not generate a plan for that command." or
        #         plan["plan_description"] == "I'm sorry, I don't know how to do that."):

    def countSpeechFailure(self, blackboard):
        self.understand_attempts += 1

        if self.understand_attempts <= 3:
            return "request_rephrase"

        self.operator_attempts += 1

        if self.operator_attempts <= 3:
            return "request_operator"

        self.instruction_count += 1
        self.understand_attempts = 0
        self.operator_attempts = 0
        return "failed"

    def checkOutcome(self, blackboard):
        steps = blackboard["steps"]
        yasmin.YASMIN_LOG_INFO(f"{steps}")

        if len(steps) == 1 and steps[0].get("skill") == "say":
            say_text = (steps[0].get("args") or {}).get("text", "")
            if say_text in ("no command given", "."):
                return "no_command"

            if say_text == PLAN_FAILED_TOKEN:
                self.understand_attempts += 1
                if self.understand_attempts <= 3:
                    return "request_rephrase"
                self.operator_attempts += 1
                if self.operator_attempts <= 3:
                    return "request_operator"
                self.instruction_count += 1
                self.understand_attempts = 0
                self.operator_attempts = 0
                return "failed"

            # Legitimate say-only response — treat as a normal plan
            return "succeeded"

        # Valid plan: reset attempt counters. The instruction counter is
        # advanced in storePlan once the plan has actually been stored.
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
            if command:
                say(self.node, f"I heard: {command}.", blackboard)
            if announcement and announcement != PLAN_FAILED_TOKEN:
                say(self.node, announcement, blackboard)
        except Exception:
            pass
        return "succeeded"

    def countSpeechFailure(self, blackboard):
        self.understand_attempts += 1

        if self.understand_attempts <= 3:
            return "request_rephrase"

        self.operator_attempts += 1

        if self.operator_attempts < 1:
            return "request_operator"

        self.instruction_count += 1
        self.understand_attempts = 0
        self.operator_attempts = 0
        return "failed"

    def storePlan(self, blackboard):
        self.plans.append(blackboard["steps"])
        self.instruction_count += 1
        yasmin.YASMIN_LOG_INFO(
            f"Stored plan {len(self.plans)} (instruction {self.instruction_count - 1})."
        )

        if self.instruction_count > 3:
            return "execute"
        return "collect_next"

    def announceAll(self, blackboard):
        return "succeeded"

    def nextPlan(self, blackboard):
        # Load the next stored plan into the blackboard for execution.
        if self.exec_index >= len(self.plans):
            return "finish"

        idx = self.exec_index
        steps = self.plans[idx]
        # Strip the leading announce say step — it was already spoken during collection
        if steps and steps[0].get("skill") == "say" and len(steps) > 1:
            steps = steps[1:]
        blackboard["steps"] = steps
        say(self.node, f"Executing command {idx + 1}.", blackboard)
        yasmin.YASMIN_LOG_INFO(f"Executing plan {idx + 1} of {len(self.plans)}.")
        self.exec_index += 1
        return "execute"

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
    #     wait_for_command = ListenState(node)

    # yasmin_ros.set_ros_loggers(node)

    # sm = yasmin.StateMachine(outcomes=["succeeded", "failed"], handle_sigint=True)

    # sm.add_state(
    #     "WAIT_FOR_COMMAND",
    #     wait_for_command,
    #     transitions={
    #         "succeeded": "SET_PLACEHOLDERS",
    #         "aborted": "WAIT_FOR_COMMAND",
    #     },
    # )

    # def _set_placeholders(blackboard):
    #     blackboard["placeholders"] = blackboard["sequence"].strip()
    #     return "succeeded"

    # set_placeholders_state = yasmin.CbState(
    #     outcomes=["succeeded"], callback=_set_placeholders
    # )
    # set_placeholders_state.add_input_key("sequence")
    # set_placeholders_state.add_output_key("placeholders")

    # sm.add_state(
    #     "SET_PLACEHOLDERS",
    #     set_placeholders_state,
    #     transitions={"succeeded": "QUERY_LLM"},
    # )

    # plan_con = yasmin.Concurrence(
    #     states={
    #         "QUERY_LLM": QueryLLM(node),
    #         "SAY_PLANNING": Say(
    #             format_str="I heard {}. Give me a moment, I am planning."
    #         ),
    #     },
    #     default_outcome="failed",
    #     outcome_map={
    #         "succeeded": {"QUERY_LLM": "succeeded"},
    #         "failed": {"QUERY_LLM": "failed"},
    #     },
    # )

    # sm.add_state(
    #     "QUERY_LLM",
    #     plan_con,
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


# TODO: logic
# When operator is ready to give command they click the button on the tablet (for now "Ready", but this might change)
# Robot then says it will listen
# listens for command, generates plan
# REpeats heard command to operator and then generated plan
# Then asks for confirmation that the command was correctly understood.
# If yes, saves plan, if no, asks for command to be rephrased
# this is repeated until 3 plans are acquired, or the corresponding rephrase limits are reached.
# THEN the commands are executed
