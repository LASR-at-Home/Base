#!/usr/bin python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from lasr_speech_recognition_interfaces.srv import TranscribeAudio  # type: ignore
from lasr_speech_recognition_interfaces.action import TranscribeSpeech

# https://docs.ros2.org/latest/api/rclpy/api/actions.html

class TestSpeechServerClient(Node):
    def __init__(self):
        Node.__init__(self, "listen_action_client")

        self.client = ActionClient(self, TranscribeSpeech, "transcribe_speech")
        self.goal_future = None
        self.result_future = None

    def send_goal(self, goal):
        self.get_logger().info("Waiting for Whisper server...")
        self.client.wait_for_server()
        self.get_logger().info("Server activated, sending goal...")

        self.goal_future = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)  # Returns a Future instance when the goal request has been accepted or rejected.
        self.goal_future.add_done_callback(self.response_cb) # When received get response

    def feedback_cb(self, msg):
        self.get_logger().info(f"Received feedback: {msg.feedback}")

    def response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().info("Goal was rejected")
            return

        self.get_logger().info("Goal was accepted")
        self.result_future = handle.get_result_async()  # Not using get_result() in cb, as can cause deadlock according to docs
        self.result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f"Transcribed Speech: {result.sequence}")

def main(args=None):
    rclpy.init(args=args)
    while rclpy.ok():
        goal = TranscribeSpeech.Goal()
        client = TestSpeechServerClient()
        try:
            client.send_goal(goal)
            rclpy.spin(client)
        except KeyboardInterrupt:
            client.get_logger().info("Shutting down...")
        finally:
            client.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
