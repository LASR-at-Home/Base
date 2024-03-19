import smach
from lasr_skills import Listen
from lasr_skills import Say

from typing import Union


class AskAndListen(smach.StateMachine):
    def __init__(self, question: Union[str, None] = None):
        if question is not None:
            smach.StateMachine.__init__(
                self,
                outcomes=["succeeded", "failed"],
                output_keys=["transcribed_speech"],
            )
        else:
            smach.StateMachine.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["tts_phrase"],
                output_keys=["transcribed_speech"],
            )

        with self:
            smach.StateMachine.add(
                "SAY",
                Say(question),
                transitions={
                    "succeeded": "LISTEN",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"text": "tts_phrase"} if question is None else {},
            )
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
