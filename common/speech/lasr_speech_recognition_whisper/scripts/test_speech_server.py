#!/usr/bin python3
from argparse import Action

import rclpy
from rclpy.action import ActionClient
from lasr_speech_recognition_interfaces.srv import TranscribeAudio, TranscribeAudioResponse  # type: ignore
from lasr_speech_recognition_interfaces.action import TranscribeSpeech

# TODO port file: action client

class TestSpeechServerClient:
    def __init__(self):
        self.node = rclpy.create_node("test_speech_server")
        self.client = ActionClient(self.node, TranscribeSpeech, "transcribe_speech")

    def send_goal(self, msg):
        goal = TranscribeSpeech.Goal()
        goal.msg = msg

        self.client.wait_for_server()
        self.client.send_goal(goal)  # should be future and async?

    # TODO add callback with future and handle result

# client.wait_for_server()
# node.get_logger().info("Done waiting")
while rclpy.ok():
    rclpy.init()
    # goal = TranscribeSpeech.Goal()
    # client.send_goal(goal)
    client = TestSpeechServerClient()
    client.send_goal(10)
    rclpy.spin(client.node)
    # client.wait_for_result()
    # result = client.get_result()
    # text = result.sequence
    text = ""
    print(f"Transcribed Speech: {text}")
