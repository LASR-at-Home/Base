#!/usr/bin/env python3

import os
import re

from typing import Dict

import rclpy
from ament_index_python import packages

from lasr_vision_yolo import yolo as yolo
from lasr_vision_yolo.yolo import AccessNode

# from src import lasr_vision_yolo as yolo
# from src import AccessNode
from sensor_msgs.msg import Image
from lasr_vision_interfaces.srv import (
    YoloDetection,
    YoloPoseDetection,
)

# TODO handle 3D detection service later on


class YoloServiceNode:
    def __init__(self):
        self.node = AccessNode.get_node()
        # Determine variables
        self.node.declare_parameter(
            "~preload", ["yolo11n.pt"]
        )  # to have a default value.. maybe there is a cleaner way to do this
        self.preload = self.node.get_parameter("~preload").value
        print(f"Preloading models: {self.preload}, type: {type(self.preload)}")

        for model in self.preload:
            yolo.load_model(model)

        # Prepare publisher
        self.debug_publishers: Dict[str, rclpy.node.Publisher] = {}
        self.debug_publisher = self.node.create_publisher(Image, "/yolo/debug", 1)

        yolo.start_tf_buffer()
        self.node.create_service(YoloDetection, "/yolo/detect", self._detect)
        self.node.create_service(YoloPoseDetection, "/yolo/detect_pose", self._detect_keypoints)
        self.node.get_logger().info("YOLO service started")

    def _detect(
        self, request: YoloDetection.Request(), response: YoloDetection.Response()
    ) -> YoloDetection.Response():
        """
        Hand off detection request to yolo library
        """
        if request.dataset in self.debug_publishers:
            debug_publisher = self.debug_publishers[request.dataset]
        else:
            topic_name = re.sub(r"[\W_]+", "", request.dataset)
            if topic_name == "":
                topic_name = "/yolo/debug"
            else:
                topic_name = f"/yolo/debug/{topic_name}"
            debug_publisher = self.node.create_publisher(Image, topic_name, 1)
        response = yolo.detect(request, debug_publisher)
        return response


def main(args=None):
    rclpy.init(args=args)

    # Put ourselves in the model folder
    package_install = packages.get_package_prefix("lasr_vision_yolo")
    package_path = os.path.abspath(
        os.path.join(
            package_install,
            os.pardir,
            os.pardir,
            "common/vision/lasr_vision_yolo",
        )
    )
    os.chdir(os.path.abspath(os.path.join(package_path, "models")))

    node = AccessNode.get_node()
    yolo_service = YoloServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.shutdown()


if __name__ == "__main__":
    main()
