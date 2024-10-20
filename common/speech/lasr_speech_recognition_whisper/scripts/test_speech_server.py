#!/usr/bin python3
from argparse import Action

import rclpy
from rclpy.action import ActionClient
from lasr_speech_recognition_interfaces.srv import TranscribeAudio, TranscribeAudioResponse  # type: ignore
from lasr_speech_recognition_interfaces.msg import (  # type: ignore
    TranscribeSpeechAction,
    TranscribeSpeechGoal,
)

# TODO port file: action client, is_shutdown

with rclpy.init(args=None):
    node = rclpy.create_node("test_speech_server")

client = ActionClient("transcribe_speech", TranscribeSpeechAction)
client.wait_for_server()
node.get_logger().info("Done waiting")
while not rospy.is_shutdown():
    goal = TranscribeSpeechGoal()
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    text = result.sequence
    print(f"Transcribed Speech: {text}")
