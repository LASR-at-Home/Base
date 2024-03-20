#!/usr/bin/env python3
import rospy
import actionlib
from lasr_speech_recognition_msgs.srv import TranscribeAudio, TranscribeAudioResponse  # type: ignore
from lasr_speech_recognition_msgs.msg import (  # type: ignore
    TranscribeSpeechAction,
    TranscribeSpeechGoal,
)


rospy.init_node("test_speech_server")
client = actionlib.SimpleActionClient("transcribe_speech", TranscribeSpeechAction)
client.wait_for_server()
rospy.loginfo("Done waiting")
while not rospy.is_shutdown():
    goal = TranscribeSpeechGoal()
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    text = result.sequence
    print(f"Transcribed Speech: {text}")
