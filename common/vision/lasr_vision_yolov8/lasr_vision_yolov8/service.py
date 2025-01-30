#!/usr/bin/env python3

import os
import re

from typing import Dict

import rclpy
from rclpy import Node
from ament_index_python import packages

import lasr_vision_yolov8 as yolo
from sensor_msgs.msg import Image
from lasr_vision_interfaces.srv import (
    YoloDetection,
)

# TODO handle 3D detection service later on

class YoloServiceNode(Node):
    def __init__(self):
        Node.__init__(self, "yolo_service_node")

        # Determine variables
        self.declare_parameter("~preload", [])  # to have a default value.. maybe there is a cleaner way to do this
        self.preload = self.get_parameter("~preload").get_parameter_value()

        for model in self.preload:
            yolo.load_model(model)

        # Prepare publisher
        self.debug_publishers: Dict[str, rclpy.node.Publisher] = {}
        self.debug_publisher = self.create_publisher(Image, "/yolov8/debug", 1)

        yolo.start_tf_buffer()
        self.create_service(YoloDetection, "/yolov8/detect", self.detect)
        self.get_logger().info("YOLOv8 service started")

    def detect(self, request: YoloDetection.Request(), response: YoloDetection.Response()) -> YoloDetection.Response():
        """
        Hand off detection request to yolo library
        """
        if request.dataset in self.debug_publishers:
            debug_publisher = self.debug_publishers[request.dataset]
        else:
            topic_name = re.sub(r"[\W_]+", "", request.dataset)
            debug_publisher = self.create_publisher(
                Image, f"/yolov8/debug/{topic_name}", 1
            )
        response = yolo.detect(request, debug_publisher)
        return response

def main(args=None):
    rclpy.init(args=args)

    # Put ourselves in the model folder
    package_install = packages.get_package_prefix("lasr_vision_yolov8")
    package_path = os.path.abspath(os.path.join(package_install, os.pardir, os.pardir, "PACKAGE_NAME", ))
    os.chdir(os.path.abspath(os.path.join(package_path, "models")))

    node = YoloServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()