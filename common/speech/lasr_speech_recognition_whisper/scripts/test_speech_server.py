#!/usr/bin python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from lasr_speech_recognition_interfaces.action import TranscribeSpeech


class TestSpeechServerClient(Node):
    def __init__(self):
        Node.__init__(self, "listen_action_client")
        self._client = ActionClient(self, TranscribeSpeech, "transcribe_speech")

    def transcribe(self) -> str:
        self.get_logger().info("Waiting for server...")
        self._client.wait_for_server()
        self.get_logger().info("Sending goal...")

        future = self._client.send_goal_async(TranscribeSpeech.Goal())
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Goal rejected")
            return ""

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.sequence


def main(args=None):
    rclpy.init(args=args)
    client = TestSpeechServerClient()
    try:
        while rclpy.ok():
            phrase = client.transcribe()
            client.get_logger().info(f"Transcription: '{phrase}'")
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
