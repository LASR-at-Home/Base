#!/usr/bin/env python3
import smach_ros
from lasr_speech_recognition_msgs.msg import (
    TranscribeSpeechAction,
    TranscribeSpeechGoal,
)


class Listen(smach_ros.SimpleActionState):
    def __init__(self):
        smach_ros.SimpleActionState.__init__(
            self,
            "transcribe_speech",
            TranscribeSpeechAction,
            goal=TranscribeSpeechGoal(),
            result_slots=["sequence"],
        )
