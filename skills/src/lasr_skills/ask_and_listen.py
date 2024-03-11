import smach
from listen import Listen
from skills.src.lasr_skills.say import Say


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
                transitions={"succeeded": "LISTEN", "failed": "failed"},
                remapping={"tts_phrase": "text"},
            )
            smach.StateMachine.add(
                "LISTEN",
                Listen(),
                transitions={"succeeded": "succeeded", "failed": "failed"},
                remapping={"transcribed_speech": "transcribed_speech"},
            )
