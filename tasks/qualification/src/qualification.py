#!/usr/bin/env python3

"""
begin in kitchen area
1. A greets robot (hi tiago)
2. Robot doesn't know who A is, so asks for their name
3. Robot learns A's face, tells them it's done
4. A asks robot to guide them to the lab 
5. Robot guides A to the lab, we mark the corners as landmarks, where the robot has to turn around and check that the person is still following. If they're not, the robot can go back around the corner and try again.
6. Robot says "we're here" and stops
7. A asks robot to find B (known person), A thinks that B needs assistance.
8. Robot finds B, asks them if they need assistance
9. B asks robot to pick up object X
10. Robot picks up object X, brings it to B
12. Robot hands object X to B

Visualisation:
- audio -> transcription -> command look-up -> action
- navigation (planning)
- face recognition
- object, person detection
- grasping
"""

import rospy

rospy.init_node("qualification")
from lasr_vision_msgs.srv import Recognise, RecogniseRequest
from sensor_msgs.msg import Image
from lasr_voice import Voice

import actionlib
from lasr_speech_recognition_msgs.srv import TranscribeAudio, TranscribeAudioResponse
from lasr_speech_recognition_msgs.msg import (
    TranscribeSpeechAction,
    TranscribeSpeechGoal,
)
import lasr_vision_deepface as face_recognition
from tiago_controllers.controllers import BaseController

import random

voice = Voice()
recognise = rospy.ServiceProxy("/recognise", Recognise)
recognise.wait_for_service()
transcribe = actionlib.SimpleActionClient("transcribe_speech", TranscribeSpeechAction)
transcribe.wait_for_server()
base_controller = BaseController()


def do_recognise(image):
    req = RecogniseRequest()
    req.image_raw = image
    req.dataset = "qualification"
    req.confidence = 0.7
    resp = recognise(req)
    return resp.detections


def do_transcribe_speech():
    transcribe.send_goal(TranscribeSpeechGoal())
    transcribe.wait_for_result()
    result = transcribe.get_result()
    return result.sequence


poses = {
    "kitchen": None,
    "lab": None,
    "lab_center": None,
}


def greet():
    # TODO: greet the person, if they are not in the database, ask them to introduce themselves and learn their face
    im = rospy.wait_for_message("/xtion/rgb/image_raw", Image)
    face_recognition_result = do_recognise(im)
    if not face_recognition_result:
        # unknown person
        voice.speak("Hello, I don't know who you are. What is your name?")
        name = do_transcribe_speech().split(" ")[
            -1
        ]  # assume the last word is the name :)
        voice.speak(f"Thank you, {name}, I will remember your face now")
        # TODO: learn face and associate with name
        face_recognition.create_dataset("/xtion/rgb/image_raw", "qualification", name)
        # just perform an inference for visualisation purposes
        face_recognition(im)
    else:
        suffixes = [
            "it's great to see you again",
            "it's nice to see you",
            "it's good to see you",
        ]
        voice.speak(
            f"Hello, {face_recognition_result[0].name} {random.choice(suffixes)}"
        )


def guide():
    # TODO: guide person to location, at landmarks, check if they are still following, if they're not, look for them
    base_controller.sync_to_pose(poses["lab"])
    pass


# Phase 1: wait for a person to greet the robot
while not rospy.is_shutdown():
    speech = do_transcribe_speech()
    if "hello" in speech or "hi" in speech:
        greet()

# Phase 2: receive command from person
voice.speak("What can I do for you?")
command = do_transcribe_speech()
if "guide" in command:
    guide()


# Phase 2: guide the person to the lab, for now just go to the lab, assume the person is following
