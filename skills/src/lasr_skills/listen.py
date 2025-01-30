#!/usr/bin/env python3
import smach_ros
from lasr_speech_recognition_interfaces.action import TranscribeSpeech


class Listen(smach_ros.SimpleActionState):
    def __init__(self):
        smach_ros.SimpleActionState.__init__(
            self,
            "transcribe_speech",
            TranscribeSpeech,
            result_slots=["sequence"],
        )
