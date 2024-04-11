import smach
from lasr_skills import Listen
from lasr_skills import Say

from typing import Union


class AskAndListen(smach.StateMachine):
    def __init__(
        self,
        tts_phrase: Union[str, None] = None,
        tts_phrase_format_str: Union[str, None] = None,
    ):

        if tts_phrase is not None:
            smach.StateMachine.__init__(
                self,
                outcomes=["succeeded", "failed"],
                output_keys=["transcribed_speech"],
            )
            with self:
                smach.StateMachine.add(
                    "SAY",
                    Say(text=tts_phrase),
                    transitions={
                        "succeeded": "LISTEN",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )
        elif tts_phrase_format_str is not None:
            smach.StateMachine.__init__(
                self,
                outcomes=["succeeded", "failed"],
                output_keys=["transcribed_speech"],
                input_keys=["tts_phrase_placeholders"],
            )
            with self:
                smach.StateMachine.add(
                    "SAY",
                    Say(format_str=tts_phrase_format_str),
                    transitions={
                        "succeeded": "LISTEN",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                    remapping={"placeholders": "tts_phrase_placeholders"},
                )
        else:
            smach.StateMachine.__init__(
                self,
                outcomes=["succeeded", "failed"],
                output_keys=["transcribed_speech"],
                input_keys=["tts_phrase"],
            )
            with self:
                smach.StateMachine.add(
                    "SAY",
                    Say(),
                    transitions={
                        "succeeded": "LISTEN",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                    remapping={"text": "tts_phrase"},
                )

        with self:
            smach.StateMachine.add(
                "LISTEN",
                Listen(),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"sequence": "transcribed_speech"},
            )
