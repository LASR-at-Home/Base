import message_filters
import yasmin
import yasmin_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy


class SaveBarPose(yasmin.State):
    def __init__(self, node, topic: str = "/amcl_pose"):
        super().__init__(outcomes=["succeeded", "failed"])
        self.node = node
        self.topic = topic

        self.robot_pose = None
        self.robot_pose_sub = yasmin_ros.logger_node.create_subscription(
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
    def robot_point_cb(self, msg: PoseWithCovarianceStamped):
        self.robot_pose = msg

    def execute(self, blackboard):
        msg = self.robot_pose
        if msg is not None:
            blackboard["bar_pose"] = msg
            yasmin.YASMIN_LOG_INFO(f"Type is {type(msg)}.")
            yasmin.YASMIN_LOG_INFO("Saved bar pose to blackboard.")
            return "succeeded"
        else:
            yasmin.YASMIN_LOG_ERROR(
                "Failed to save bar pose. No pose message received."
            )
            return "failed"
