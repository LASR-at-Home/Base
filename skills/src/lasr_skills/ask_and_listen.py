import smach
from lasr_skills import Listen
from lasr_skills import Say

from typing import Union


class AskAndListen(smach.StateMachine):
    def __init__(
        self,
        node,
        tts_phrase: Union[str, None] = None,
        tts_phrase_format_str: Union[str, None] = None,
    ):
        if tts_phrase is not None:
            super().__init__(
                outcomes=["succeeded", "failed"],
                output_keys=["transcribed_speech"],
            )
            with self:
                self.add(
                    "SAY",
                    Say(node=node, text=tts_phrase),
                    transitions={
                        "succeeded": "LISTEN",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )
                self.add(
                    "LISTEN",
                    Listen(node=node),
                    transitions={
                        "succeeded": "succeeded",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                    remapping={"sequence": "transcribed_speech"},
                )
        elif tts_phrase_format_str is not None:
            super().__init__(
                outcomes=["succeeded", "failed"],
                output_keys=["transcribed_speech"],
                input_keys=["tts_phrase_placeholders"],
            )
            with self:
                self.add(
                    "SAY",
                    Say(node=node, format_str=tts_phrase_format_str),
                    transitions={
                        "succeeded": "LISTEN",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                    remapping={"placeholders": "tts_phrase_placeholders"},
                )
                self.add(
                    "LISTEN",
                    Listen(node=node),
                    transitions={
                        "succeeded": "succeeded",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                    remapping={"sequence": "transcribed_speech"},
                )
        else:
            super.__init__(
                outcomes=["succeeded", "failed"],
                output_keys=["transcribed_speech"],
                input_keys=["tts_phrase"],
            )
            with self:
                self.add(
                    "SAY",
                    Say(node=node),
                    transitions={
                        "succeeded": "LISTEN",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                    remapping={"text": "tts_phrase"},
                )
                self.add(
                    "LISTEN",
                    Listen(node=node),
                    transitions={
                        "succeeded": "succeeded",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                    remapping={"sequence": "transcribed_speech"},
                )
