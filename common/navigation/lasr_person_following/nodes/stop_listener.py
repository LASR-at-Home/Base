#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.action import ActionClient

from lasr_speech_recognition_msgs.action import TranscribeSpeech
from std_msgs.msg import Empty

from typing import Optional, List


class StopListener(Node):

    _finished_pub: Publisher
    _transcribe_speech_client: ActionClient

    def __init__(self) -> None:
        super().__init__("stop_listener")
        self._finished_pub = self.create_publisher(Empty, "/stop_listener/finished")
        self._transcribe_speech_client = ActionClient(
            self, TranscribeSpeech, "transcribe_speech"
        )

    def run(self) -> None:
        while True:
            self._transcribe_speech_client.wait_for_server()
            future: ActionClient.Future = (
                self._transcribe_speech_client.send_goal_async(TranscribeSpeech.Goal())
            )
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if "stop" in result.sequence.lower():
                self._finished_pub.publish(Empty())


def main(args: Optional[List[str]] = None):
    rclpy.init(args=args)
    stop_listener_node: StopListener = StopListener()
    rclpy.spin(stop_listener_node)
    rclpy.shutdown()
