#!/usr/bin/env python3
import rospy
import actionlib
from lasr_voice import Voice  # type: ignore
from lasr_speech_recognition_msgs.srv import TranscribeAudio, TranscribeAudioResponse  # type: ignore
from lasr_speech_recognition_msgs.msg import (  # type: ignore
    TranscribeSpeechAction,
    TranscribeSpeechGoal,
)

# import actionlib
rospy.init_node("repeat")

USE_ACTIONLIB = True

voice = Voice()


if USE_ACTIONLIB:
    client = actionlib.SimpleActionClient("transcribe_speech", TranscribeSpeechAction)
    client.wait_for_server()
    repeating = False
    rospy.loginfo("Done waiting")
    while not rospy.is_shutdown():
        goal = TranscribeSpeechGoal()
        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_result()
        text = result.sequence
        print(text)
        if "tiago" in text.lower().strip():
            if "repeat" in text.lower().strip():
                repeating = True
                voice.sync_tts("Okay, I'll start repeating now.")
                continue
            elif "stop" in text.lower().strip():
                repeating = False
                voice.sync_tts("Okay, I'll stop repeating now.")
                break
        if repeating:
            voice.sync_tts(f"I heard {text}")
else:
    transcribe = rospy.ServiceProxy("/whisper/transcribe_audio", TranscribeAudio)
    repeating = False
    while not rospy.is_shutdown():
        text = transcribe().phrase
        print(text)
        if "tiago" in text.lower().strip():
            if "repeat" in text.lower().strip():
                repeating = True
                voice.sync_tts("Okay, I'll start repeating now.")
                continue
            elif "stop" in text.lower().strip():
                repeating = False
                voice.sync_tts("Okay, I'll stop repeating now.")
                break
        if repeating:
            voice.sync_tts(f"I heard {text}")
