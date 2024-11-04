# #!/usr/bin python3
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from lasr_voice import Voice  # type: ignore
# from lasr_speech_recognition_interfaces.srv import TranscribeAudio, TranscribeAudioResponse  # type: ignore
# from lasr_speech_recognition_interfaces.action import TranscribeSpeechAction, TranscribeSpeechGoal
#
#
# # TODO port file: action client, service proxy
#
# USE_ACTIONLIB = True
#
# class ServiceClientNode(Node):
#     def __init__(self):
#         super().__init__('service_client_node')
#         self.client = None
#
#         self.voice = Voice()
#
#     # def call_service(self):
#     #     request = TranscribeAudio
#     #
#     #     # Call the service synchronously
#     #     future = self.client.call_async(request)
#     #     rclpy.spin_until_future_complete(self, future)
#     #
#     #     if future.result() is not None:
#     #         self.get_logger().info('Service call succeeded')
#     #     else:
#     #         self.get_logger().error('Service call failed')
#
#
#     def call_service(self):
#         request = TranscribeAudio
#         else:
#             # transcribe = rospy.ServiceProxy("/whisper/transcribe_audio", TranscribeAudio)
#             repeating = False
#             while rclpy.ok():
#                 future = self.client.call_async(request)
#                 rclpy.spin_until_future_complete(self, future)
#                 if future.done():
#                     text = transcribe().phrase
#                     self.get_logger().info(text)
#                     if "tiago" in text.lower().strip():
#                         if "repeat" in text.lower().strip():
#                             repeating = True
#                             self.voice.sync_tts("Okay, I'll start repeating now.")
#                             continue
#                         elif "stop" in text.lower().strip():
#                             repeating = False
#                             self.voice.sync_tts("Okay, I'll stop repeating now.")
#                             break
#                     if repeating:
#                         self.voice.sync_tts(f"I heard {text}")
#
#
# class ActionClientNode(Node):
#     def __init__(self):
#         super().__init__('action_client_node')
#         self.client = None
#
#     def goal_callback(self, future):
#         self.client = self.create_client(TranscribeSpeechAction, "transcribe_speech")
#         # Wait for the server to be available
#         while not self.client.wait_for_service(timeout_sec=5.0):
#             self.get_logger().info("Waiting for server...")
#         repeating = False
#         self.get_logger().info("Done waiting")
#         while rclpy.ok():
#             goal = TranscribeSpeechGoal()
#             self.client.send_goal(goal)
#             self.client.wait_for_result()
#             result = self.client.get_result()
#             text = result.sequence
#             self.get_logger().info(text)
#             if "tiago" in text.lower().strip():
#                 if "repeat" in text.lower().strip():
#                     repeating = True
#                     self.voice.sync_tts("Okay, I'll start repeating now.")
#                     continue
#                 elif "stop" in text.lower().strip():
#                     repeating = False
#                     self.voice.sync_tts("Okay, I'll stop repeating now.")
#                     break
#             if repeating:
#                 self.voice.sync_tts(f"I heard {text}")