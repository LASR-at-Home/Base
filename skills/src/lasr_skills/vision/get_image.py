import yasmin_ros
import yasmin
from yasmin import State, StateMachine

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


from typing import Optional
from sensor_msgs.msg import Image, PointCloud2

import time


class GetImage(State):
    """
    State for reading an sensor_msgs Image message
    """

    def __init__(self, topic="head_front_camera/rgb/image_raw"):
        super().__init__(outcomes=["succeeded", "failed"])

        self.add_input_key("img_msg")
        self.add_output_key("img_msg")

        self.camera_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.node = yasmin_ros.logger_node

        self.msg = None

        self.node.create_subscription(
            Image, topic, self.image_cb, qos_profile=self.camera_qos
        )

    def image_cb(self, msg):
        if self.msg is None:
            self.msg = msg

    def execute(self, blackboard):
        self.msg = None

        while self.msg is None:
            yasmin.YASMIN_LOG_INFO("Waiting for rgb frame")
            time.sleep(1)

        try:
            blackboard["img_msg"] = self.msg
            return "succeeded"
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(str(e))
            return "failed"


# UNUSED THROUGHOUT WHOLE REPO, MAYBE DELETE?????


class GetPointCloud(State):
    """
    State for acquiring a PointCloud2 message.
    """

    def __init__(self, topic: Optional[str] = None):
        super().__init__(outcomes=["succeeded", "failed"])

        self.add_input_key("pcl_msg")
        self.add_output_key("pcl_msg")

        self.camera_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        yasmin_ros.logger_node.declare_parameter(
            "image_topic", "/head_front_camera/rgb/image_raw"
        )
        self.topic = (
            topic
            if topic
            else yasmin_ros.logger_node.get_parameter("image_topic")
            .get_parameter_value()
            .string_value
        )

    def execute(self, blackboard):
        # if not rclpy.ok():
        #     rclpy.init()
        try:
            blackboard["pcl_msg"] = None
            blackboard["pcl_msg"] = wait_for_message(
                PointCloud2,
                yasmin_ros.logger_node,
                self.topic,
                qos_profile=self.camera_qos,
            )
            if blackboard["pcl_msg"] is None:
                return "failed"
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(str(e))
            return "failed"
        return "succeeded"


# ALSO NEVER USED, MAYBE DELETE AS WELL????
class GetImageAndPointCloud(State):
    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("pcl_msg")
        self.add_input_key("img_msg")

        self.add_output_key("pcl_msg")
        self.add_output_key("img_msg")

        self.camera_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.topic1 = "/head_front_camera/rgb/image_raw"
        self.topic2 = "/head_front_camera/depth/points"

        self.topic1 = "/head_front_camera/rgb/image_raw"
        self.topic2 = "/head_front_camera/depth/points"

    def execute(self, blackboard):
        # if not rclpy.ok():
        #     rclpy.init()
        try:
            blackboard["img_msg"] = wait_for_message(
                Image, yasmin_ros.logger_node, self.topic1, self.camera_qos
                Image, yasmin_ros.logger_node, self.topic1, self.camera_qos
            )
            blackboard["pcl_msg"] = wait_for_message(
                PointCloud2, yasmin_ros.logger_node, self.topic2, self.camera_qos
                PointCloud2, yasmin_ros.logger_node, self.topic2, self.camera_qos
            )

            if blackboard["img_msg"] is None or blackboard["pcl_msg"] is None:
                return "failed"
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(str(e))
            return "failed"

        return "succeeded"
