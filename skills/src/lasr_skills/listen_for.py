import smach_ros
from lasr_speech_recognition_msgs.msg import (
    TranscribeSpeechAction,
    TranscribeSpeechGoal,
)

from actionlib_msgs.msg import GoalStatus


class ListenFor(smach_ros.SimpleActionState):
    def __init__(self, wake_word: str):

        def speech_result_cb(userdata, status, result):
            if status == GoalStatus.SUCCEEDED:
                if wake_word in result.transcription.lower().split():
                    return "succeeded"
                return "not_done"
            return "failed"

        smach_ros.SimpleActionState.__init__(
            self,
            "transcribe_speech",
            TranscribeSpeechAction,
            result_cb=speech_result_cb,
        )
