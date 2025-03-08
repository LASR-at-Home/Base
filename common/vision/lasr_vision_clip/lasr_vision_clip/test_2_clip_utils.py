#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
import threading
import cv_bridge

from sensor_msgs.msg import Image
from lasr_vision_interfaces.srv import Vqa


class ClipCameraTester(Node):
    def __init__(self, listen_topic, possible_answers):
        super().__init__("clip_camera_tester")

        self.listen_topic = listen_topic
        self.possible_answers = possible_answers
        self.processing = False

        self.vqa_client = self.create_client(Vqa, "clip_vqa/query_service")

        while not self.vqa_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Waiting for CLIP VQA service...")

        self.subscription = self.create_subscription(
            Image, self.listen_topic, self.image_callback, 10
        )

        self.bridge = cv_bridge.CvBridge()
        self.get_logger().info(f"Listening for images on topic: {self.listen_topic}")

    def query_clip(self, image):
        self.processing = True
        self.get_logger().info("Processing image with CLIP...")

        req = Vqa.Request()
        req.image_raw = image
        req.possible_answers = self.possible_answers

        future = self.vqa_client.call_async(req)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f"CLIP Response: Answer={response.answer}, Similarity={response.similarity}"
            )
        except Exception as e:
            self.get_logger().error(f"CLIP service call failed: {e}")
        finally:
            self.processing = False

    def image_callback(self, image_msg):
        if self.processing:
            return

        threading.Thread(target=self.query_clip, args=(image_msg,)).start()


def main(args=None):
    rclpy.init(args=args)

    listen_topic = "/xtion/rgb/image_raw"
    possible_answers = ["Wearing glasses", "Not wearing glasses"]

    node = ClipCameraTester(listen_topic, possible_answers)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
