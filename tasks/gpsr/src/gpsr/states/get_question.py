#!/usr/bin/env python3
import smach
import rospy
import actionlib
from lasr_voice import Voice
from lasr_speech_recognition_msgs.msg import (
    TranscribeSpeechAction,
    TranscribeSpeechGoal,
)


class GetQuestion(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=["succeeded", "failed"], output_keys=["question"]
        )
        self.voice = Voice()
        self.client = actionlib.SimpleActionClient(
            "transcribe_speech", TranscribeSpeechAction
        )

    def execute(self, userdata):
        try:
            self.client.wait_for_server()
            self.voice.sync_tts("Hello, I hear you have a question for me, ask away!")
            goal = TranscribeSpeechGoal()
            self.client.send_goal(goal)
            self.client.wait_for_result()
            result = self.client.get_result()
            text = result.sequence
            userdata.question = text
            return "succeeded"
        except Exception as e:
            rospy.loginfo(f"Failed to get question: {e}")
            return "failed"
