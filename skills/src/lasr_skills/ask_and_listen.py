import smach
from lasr_skills import Listen
from lasr_skills import Say


class AskAndListen(smach.StateMachine):
    def __init__(self):
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
