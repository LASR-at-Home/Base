import yasmin
import rclpy
import yasmin_ros
import rclpy
import yasmin_ros
from lasr_skills import Listen
from lasr_skills import Say

from typing import Union


class AskAndListen(yasmin.StateMachine):
    def __init__(
        self,
        tts_phrase: Union[str, None] = None,
        tts_phrase_format_str: Union[str, None] = None,
    ):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)
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
            self.add_state(
                "LISTEN",
                Listen(),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "canceled": "failed",
                },
                remappings={"sequence": "transcribed_speech"},
            )
        elif tts_phrase_format_str is not None:
            self.add_input_key("tts_phrase_placeholders")

            self.add_state(
                "SAY",
                Say(format_str=tts_phrase_format_str),
                transitions={
                    "succeeded": "LISTEN",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remappings={"placeholders": "tts_phrase_placeholders"},
            )
            self.add_state(
                "LISTEN",
                Listen(),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remappings={"sequence": "transcribed_speech"},
            )
        else:
            self.add_input_key("tts_phrase")
            self.add_state(
                "SAY",
                Say(),
                transitions={
                    "succeeded": "LISTEN",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"text": "tts_phrase"},
            )
            self.add(
                "LISTEN",
                Listen(),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"sequence": "transcribed_speech"},
            )


def main():
    rclpy.init()

    yasmin_ros.set_ros_loggers()

    sm = AskAndListen("PLease say hi tiago then say your name and favourite drink")

    outcome = sm()

    yasmin.YASMIN_LOG_INFO(f"SM FINISHED WITH OUTCOME {outcome}")

    rclpy.shutdown()
