import rclpy
from rclpy.action import ActionClient

from lasr_vision_interfaces.action import EyeTracker as EyeTrackerAction


class StartEyeTracker:
    def __init__(self, node):
        self.node = node

        self._action_client = ActionClient(
            self.node,
            EyeTrackerAction,
            "/lasr_vision_eye_tracker/track_eyes",
        )

        self.node.get_logger().info("Waiting for eye tracker action server...")
        self._action_client.wait_for_server()
        self.node.get_logger().info("Eye tracker action server is available.")

    def execute(self, userdata):
        goal_msg = EyeTrackerAction.Goal()
        goal_msg.person_point = userdata.person_point

        self.node.get_logger().info("Sending eye tracker goal...")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        return "succeeded"





# --------------------------------

class StopEyeTracker:
    def __init__(self, node):
        self.node = node
        self._action_client = ActionClient(
            self.node,
            EyeTrackerAction,
            "/lasr_vision_eye_tracker/track_eyes",
        )

    def execute(self, userdata):
        pass
    





# import smach
# import rclpy
# from rclpy.action import ActionClient

# from lasr_vision_interfaces.action import EyeTracker as EyeTrackerAction


# class StartEyeTracker(smach.State):
#     def __init__(self, node):
#         smach.State.__init__(
#             self,
#             outcomes=["succeeded", "failed"],
#             input_keys=["person_point"],
#             output_keys=["eye_tracker_goal_handle"],
#         )

#         self.node = node
#         self._action_client = ActionClient(
#             self.node,
#             EyeTrackerAction,
#             "/lasr_vision_eye_tracker/track_eyes",
#         )

#         self.node.get_logger().info("Waiting for eye tracker action server...")
#         self._action_client.wait_for_server()
#         self.node.get_logger().info("Eye tracker action server is available.")

#     def execute(self, userdata):
#         goal_msg = EyeTrackerAction.Goal()
#         goal_msg.person_point = userdata.person_point

#         self.node.get_logger().info("Sending eye tracker goal...")
#         send_goal_future = self._action_client.send_goal_async(goal_msg)

#         rclpy.spin_until_future_complete(self.node, send_goal_future)
#         goal_handle = send_goal_future.result()

#         if goal_handle is None:
#             self.node.get_logger().error("No goal handle returned from eye tracker.")
#             return "failed"

#         if not goal_handle.accepted:
#             self.node.get_logger().warn("Eye tracker goal was rejected.")
#             return "failed"

#         self.node.get_logger().info("Eye tracker goal accepted.")
#         userdata.eye_tracker_goal_handle = goal_handle
#         return "succeeded"


# class StopEyeTracker(smach.State):
#     def __init__(self, node):
#         smach.State.__init__(
#             self,
#             outcomes=["succeeded", "failed"],
#             input_keys=["eye_tracker_goal_handle"],
#             output_keys=["eyes"],
#         )

#         self.node = node

#     def execute(self, userdata):
#         goal_handle = getattr(userdata, "eye_tracker_goal_handle", None)

#         if goal_handle is None:
#             self.node.get_logger().warn("No active eye tracker goal handle found.")
#             return "failed"

#         self.node.get_logger().info("Cancelling eye tracker goal...")
#         cancel_future = goal_handle.cancel_goal_async()

#         rclpy.spin_until_future_complete(self.node, cancel_future)
#         cancel_response = cancel_future.result()

#         if cancel_response is None:
#             self.node.get_logger().error("No cancel response received.")
#             return "failed"

#         if len(cancel_response.goals_canceling) > 0:
#             self.node.get_logger().info("Eye tracker goal cancelled successfully.")
#             userdata.eye_tracker_goal_handle = None
#             return "succeeded"

#         self.node.get_logger().warn("Eye tracker goal was not cancelled.")
#         return "failed"