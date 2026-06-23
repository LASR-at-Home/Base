import yasmin
import yasmin_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

import time


class SaveBarPose(yasmin.State):
    def __init__(self, topic: str = "/amcl_pose"):
        super().__init__(outcomes=["succeeded", "failed"])
        self.topic = topic
        self.node = yasmin_ros.logger_node

        self.add_output_key("bar_pose")

        self.robot_pose = None
        self.robot_pose_sub = self.node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.robot_point_cb,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )
        time.sleep(1)

    def robot_point_cb(self, msg: PoseWithCovarianceStamped):
        self.robot_pose = msg.pose.pose

    def execute(self, blackboard):
        msg = self.robot_pose
        try:
            blackboard["bar_pose"] = msg
            yasmin.YASMIN_LOG_INFO(f"Type is {type(msg)}.")
            yasmin.YASMIN_LOG_INFO("Saved bar pose to blackboard.")
            if msg is None:
                yasmin.YASMIN_LOG_INFO(f"Message has value {msg}.")
                return "failed"
            return "succeeded"
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(
                f"Failed to save bar pose. No pose message received. {e}"
            )
            return "failed"
