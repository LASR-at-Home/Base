import message_filters
import yasmin
from geometry_msgs.msg import PoseWithCovarianceStamped


class SaveBarPose(yasmin.State):
    def __init__(self, node, topic: str = "/amcl_pose"):
        super().__init__(outcomes=["succeeded", "failed"])
        self.node = node
        self.topic = topic
        pose_info = message_filters.Subscriber(
            self.node, PoseWithCovarianceStamped, self.topic
        )

        self.cache = message_filters.Cache(pose_info)

    def execute(self, blackboard):
        msg = self.cache.getLast()
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
