#!/usr/bin/env python3

import sys
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from lasr_vision_interfaces.srv import BodyPixMaskDetection


class MaskRelay(Node):
    def __init__(self):
        super().__init__("image_listener")
        self.declare_parameter("image_topic", "/head_front_camera/rgb/image_raw")
        self.listen_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.processing = False

        # Set up the service client
        self.detect_service_client = self.create_client(
            BodyPixMaskDetection, "/bodypix/mask_detection"
        )
        while not self.detect_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")
        
        # Set up the subscriber
        self.subscription = self.create_subscription(
            Image, self.listen_topic, self.image_callback, 10  # QoS profile
        )
        self.get_logger().info(f"Started listening on topic: {self.listen_topic}")

    def detect(self, image):
        self.processing = True
        self.get_logger().info("Received image message")
        # Create a request for the service
        req = BodyPixMaskDetection.Request()
        req.image_raw = image
        # req.dataset = self.model
        req.confidence = 0.7
        req.parts = ["left_face", "right_face"]

        # Call the service asynchronously
        future = self.detect_service_client.call_async(req)
        future.add_done_callback(self.detect_callback)

    def detect_callback(self, future):
        try:
            response = future.result()
            if response is not None:
                # Modify masks for demonstration purposes
                for mask in response.masks:
                    mask.mask = [True, False, True, False]
                self.get_logger().info(f"Detection response received: {response}")
            else:
                self.get_logger().error("Service call returned no response")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        finally:
            self.processing = False

    def image_callback(self, image):
        if self.processing:
            return

        # Start a new thread for detection to avoid blocking
        threading.Thread(target=self.detect, args=(image,)).start()


def main(args=None):
    print("Starting mask_relay node")

    rclpy.init(args=args)
    mask_relay_node = MaskRelay()
    mask_relay_node.get_logger().info("Mask relay node started")

    try:
        rclpy.spin(mask_relay_node)
    except KeyboardInterrupt:
        pass
    finally:
        mask_relay_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
