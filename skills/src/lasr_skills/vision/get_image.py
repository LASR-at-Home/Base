import yasmin_ros
import yasmin
from yasmin import State, StateMachine

import rclpy
from rclpy.wait_for_message import wait_for_message

from typing import Optional
from sensor_msgs.msg import Image, PointCloud2


class GetImage(State):
    """
    State for reading an sensor_msgs Image message
    """

    def __init__(self, topic: Optional[str] = None):
        super().__init__(outcomes=["succeeded", "failed"])

        self.add_input_key("img_msg")
        self.add_output_key("img_msg")

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
            msg = wait_for_message(Image, yasmin_ros.logger_node, self.topic)
            if msg is not None:
                blackboard["img_msg"] = msg
            else:
                blackboard["img_msg"] = None
            if blackboard["img_msg"] is None:
                return "failed"

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(str(e))
            return "failed"
        return "succeeded"


class GetPointCloud(State):
    """
    State for acquiring a PointCloud2 message.
    """

    def __init__(self, topic: Optional[str] = None):
        super().__init__(outcomes=["succeeded", "failed"])

        self.add_input_key("pcl_msg")
        self.add_output_key("pcl_msg")

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
                PointCloud2, yasmin_ros.logger_node, self.topic
            )
            if blackboard["pcl_msg"] is None:
                return "failed"
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(str(e))
            return "failed"
        return "succeeded"


class GetImageAndPointCloud(State):
    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("pcl_msg")
        self.add_input_key("img_msg")

        self.add_output_key("pcl_msg")
        self.add_output_key("img_msg")

        self.topic1 = "/head_front_camera/rgb/image_raw"
        self.topic2 = "/head_front_camera/depth/points"

        self.topic1 = "/head_front_camera/rgb/image_raw"
        self.topic2 = "/head_front_camera/depth/points"

    def execute(self, blackboard):
        # if not rclpy.ok():
        #     rclpy.init()
        try:
            blackboard["img_msg"] = wait_for_message(
                Image, yasmin_ros.logger_node, self.topic1
            )
            blackboard["pcl_msg"] = wait_for_message(
                PointCloud2, yasmin_ros.logger_node, self.topic2
            )

            if blackboard["img_msg"] is None or blackboard["pcl_msg"] is None:
                return "failed"
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(str(e))
            return "failed"

        return "succeeded"
