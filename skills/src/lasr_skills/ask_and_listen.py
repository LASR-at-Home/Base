import yasmin
import rclpy
import yasmin_ros
from yasmin import State
from lasr_skills import Listen
from lasr_skills import Say

from typing import Union

RETRY_PHRASE_1 = "Sorry, I didn't catch that, could you please repeat?"
RETRY_PHRASE_2 = (
    "Sorry, I still couldn't hear you, could you please repeat more loudly?"
)


class CheckSpeechState(State):
    def __init__(self):
        super().__init__(outcomes=["succeeded", "empty"])

    def execute(self, blackboard):
        transcribed_speech = blackboard["transcribed_speech"]
        if transcribed_speech and transcribed_speech.strip():
            return "succeeded"
        return "empty"


class AskAndListen(yasmin.StateMachine):
    def __init__(
        self,
        tts_phrase: Union[str, None] = None,
        tts_phrase_format_str: Union[str, None] = None,
    ):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_output_key("transcribed_speech")
        if tts_phrase is not None:
            self.add_state(
                "SAY",
                Say(text=tts_phrase),
                transitions={
                    "succeeded": "LISTEN",
                    "aborted": "failed",
                    "canceled": "failed",
                },
            )
            self._add_listen_with_retries(
                hard_failure_outcome="canceled",
            )
        elif tts_phrase_format_str is not None:
            self.add_input_key("tts_phrase_placeholders")

            self.add_state(
                "SAY",
                Say(format_str=tts_phrase_format_str),
                transitions={
                    "succeeded": "LISTEN",
                    "aborted": "failed",
                    "canceled": "failed",
                },
                remappings={"placeholders": "tts_phrase_placeholders"},
            )
            self._add_listen_with_retries(
                hard_failure_outcome="canceled",
            )
        else:
            self.add_input_key("tts_phrase")
            self.add_state(
                "SAY",
                Say(),
                transitions={
                    "succeeded": "LISTEN",
                    "aborted": "failed",
                    "canceled": "failed",
                },
                remappings={"text": "tts_phrase"},
            )
            self._add_listen_with_retries(
                hard_failure_outcome="canceled",
            )

    def _add_listen_with_retries(
        self,
        hard_failure_outcome: str,
    ):
        """
        Adds a LISTEN state, plus 2 retries (3 attempts total), each of
        which checks transcribed_speech for emptiness and asks the user to
        repeat themselves, with an escalating phrase, if nothing was heard.

        hard_failure_outcome: "canceled" or "preempted" outcomes from Say or Listen
        """
        retry_phrases = [RETRY_PHRASE_1, RETRY_PHRASE_2]
        num_attempts = len(retry_phrases) + 1

        for attempt in range(1, num_attempts + 1):
            listen_name = "LISTEN" if attempt == 1 else f"LISTEN_RETRY_{attempt - 1}"
            check_name = f"CHECK_SPEECH_{attempt}"
            next_say = f"SAY_RETRY_{attempt}"
            is_last = attempt == num_attempts

            self.add_state(
                listen_name,
                Listen(),
                transitions={
                    "succeeded": check_name,
                    "aborted": "failed" if is_last else next_say,
                    hard_failure_outcome: "failed",
                },
                **{"remappings": {"sequence": "transcribed_speech"}},
            )
            self.add_state(
                check_name,
                CheckSpeechState(),
                transitions={
                    "succeeded": "succeeded",
                    "empty": "failed" if is_last else next_say,
                },
            )
            if not is_last:
                self.add_state(
                    next_say,
                    Say(text=retry_phrases[attempt - 1]),
                    transitions={
                        "succeeded": f"LISTEN_RETRY_{attempt}",
                        "aborted": "failed",
                        hard_failure_outcome: "failed",
                    },
                )


def main():
    rclpy.init()

    yasmin_ros.set_ros_loggers()

    sm = AskAndListen("PLease say hi tiago then say your name and favourite drink")

    outcome = sm()

    yasmin.YASMIN_LOG_INFO(f"SM FINISHED WITH OUTCOME {outcome}")

    rclpy.shutdown()
