#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
import threading

from sensor_msgs.msg import Image
from lasr_vision_interfaces.srv import BodyPixKeypointDetection


class KeypointRelay(Node):
    def __init__(
        self,
        listen_topic,
    ):
        super().__init__("keypoint_relay")
        self.listen_topic = listen_topic
        self.processing = False

        # Set up the service client
        self.detect_service_client = self.create_client(
            BodyPixKeypointDetection, "/bodypix/keypoint_detection"
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
        req = BodyPixKeypointDetection.Request()
        req.image_raw = image
        # req.dataset = self.model
        req.confidence = 0.7

        # Call the service asynchronously
        future = self.detect_service_client.call_async(req)
        future.add_done_callback(self.detect_callback)

    def detect_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Detection response: {response}")
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
    print("start keypoint_relay")
    # Check if command-line arguments are sufficient
    if len(sys.argv) < 2:
        print(
            "Usage: ros2 run lasr_vision_bodypix keypoint_relay.py <source_topic> [resnet50|mobilenet50|...]"
        )
        sys.exit(1)

    # Parse the command-line arguments

    listen_topic = "/head_front_camera/rgb/image_raw"
    # if isinstance(sys.argv[1], list):
    #     listen_topic = sys.argv[1][0]

    rclpy.init(args=args)
    keypoint_relay_node = KeypointRelay(
        listen_topic,
    )
    keypoint_relay_node.get_logger().info("Keypoint relay node started")

    try:
        rclpy.spin(keypoint_relay_node)
    except KeyboardInterrupt:
        pass
    finally:
        keypoint_relay_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
