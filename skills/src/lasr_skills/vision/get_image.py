import yasmin_ros
import yasmin
from yasmin import State, StateMachine

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import message_filters
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

        self.node = yasmin_ros.logger_node

        camera_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.image_sub = message_filters.Subscriber(
            self.node, Image, "head_front_camera/rgb/image_raw", camera_qos
        )

        self.cache = message_filters.Cache(self.image_sub)

    def execute(self, blackboard):
        try:
            blackboard["img_msg"] = self.cache.getLast()
            return "succeeded"
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(str(e))
            return "failed"