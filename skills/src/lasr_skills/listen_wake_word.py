import smach
from smach_ros import SimpleActionState

from lasr_speech_recognition_msgs.msg import (
    TranscribeSpeechAction,
    TranscribeSpeechGoal,
)

from actionlib_msgs.msg import GoalStatus


class ListenWakeWord(smach.StateMachine):
    def __init__(self, wake_word: str):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:

            def listen_cb(_, status, result):
                if status == GoalStatus.SUCCEEDED:
                    if wake_word in result.transcription.lower().split():
                        return "succeeded"
                return "failed"

            smach.StateMachine.add(
                "LISTEN",
                SimpleActionState(
                    "transcribe_speech",
                    TranscribeSpeechAction,
                    goal=TranscribeSpeechGoal(),
                    result_cb=listen_cb,
                ),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
