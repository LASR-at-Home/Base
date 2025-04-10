#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node

from lasr_vision_cropped_detection import process_detection_requests
from lasr_vision_msgs.srv import CroppedDetection


CroppedDetection_Request = CroppedDetection.Request()
CroppedDetection_Response = CroppedDetection.Response()


class CroppedDetectionService(Node):
    def __init__(self):
        super().__init__("cropped_detection_service")

        self.declare_parameter("image_topic", "/head_front_camera/rgb/image_raw")
        self.listen_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )

        self.create_service(CroppedDetection, "/vision/cropped_detection", self.cropped_detection_callback)
        self.get_logger().info("Cropped Detection service started")

    def cropped_detection_callback(self, request, response):
        self.get_logger().info("Received cropped detection request, dispatching...")

        response = process_detection_requests(self, request, rgb_image_topic=self.listen_topic)

        self.get_logger().info("Cropped detection request processed")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CroppedDetectionService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
