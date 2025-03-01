#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
import threading

from sensor_msgs.msg import Image
from lasr_vision_interfaces.srv import YoloDetection

class ImageListener(Node):
    def __init__(self, listen_topic, model):
        super().__init__('image_listener')

        self.listen_topic = listen_topic
        self.model = model
        self.processing = False

        self.detect_client = self.create_client(YoloDetection, '/yolov8/detect')

        while not self.detect_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Waiting for /yolov8/detect service")

        self.subscription = self.create_subscription(Image, self.listen_topic, self.image_callback,10)

        self.get_logger().info(f"Listening on topic: {self.listen_topic}")

    def detect(self, image):
        self.processing = True
        self.get_logger().info("Received image message")

        req = YoloDetection.Request()
        req.image_raw = image
        req.dataset = self.model
        req.confidence = 0.25
        req.nms = 0.4

        # Call the service asynchronously
        future = self.detect_client.call_async(req)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Detection Result: {response}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        finally:
            self.processing = False

    def image_callback(self, image):
        if self.processing:
            return

        t = threading.Thread(target=self.detect, args=(image,))
        t.start()


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print('Usage: ros2 run lasr_objcet_detection_yolov8 relay <source_topic> [model.pt]')
        return

    listen_topic = sys.argv[1]
    model = sys.argv[2] if len(sys.argv) >= 3 else "yolov8n.pt"

    node = ImageListener(listen_topic, model)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
