#!/usr/bin/env python3
import smach_ros
from lasr_speech_recognition_interfaces.action import TranscribeSpeech


class Listen(smach_ros.SimpleActionState):
    def __init__(
        self,
        node,
    ):
        super().__init__(
            self, node, "transcribe_speech", TranscribeSpeech, result_slots=["sequence"]
        )
