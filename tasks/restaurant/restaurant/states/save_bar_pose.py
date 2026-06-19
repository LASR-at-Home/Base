import rclpy
import yasmin
from geometry_msgs.msg import PoseWithCovarianceStamped

class SaveBarPose(yasmin.State):
    def __init__(self, node, topic: str = "/amcl_pose", timeout: float = 5.0):
        super().__init__(outcomes=["succeeded", "failed"])
        self.node = node
        self.topic = topic
        self.timeout = timeout

    def execute(self, blackboard):
        try:
            success, msg = rclpy.wait_for_message.wait_for_message(
                msg_type=PoseWithCovarianceStamped,
                node=self.node,
                topic=self.topic,
                time_to_wait=self.timeout,
            )
            if success and msg is not None:
                blackboard["bar_pose"] = msg.pose.pose
                yasmin.YASMIN_LOG_INFO("Saved bar pose to blackboard.")
                return "succeeded"
            else:
                blackboard["bar_pose"] = None
                yasmin.YASMIN_LOG_WARN(
                    "No pose received from /amcl_pose, saved None to blackboard."
                )
                return "failed"
        except Exception as exc:
            yasmin.YASMIN_LOG_ERROR(f"Failed to save bar pose: {exc}")
            return "failed"
