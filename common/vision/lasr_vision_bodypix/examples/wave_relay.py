#!/usr/bin/env python3

import sys
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from lasr_vision_interfaces.srv import BodyPixWaveDetection


class WaveRelay(Node):
    def __init__(self, listen_image_topic, listen_pcl_topic):
        super().__init__("image_listener")
        self.listen_image_topic = listen_image_topic
        self.listen_pcl_topic = listen_pcl_topic
        self.processing = False

        # Set up the service client
        self.detect_service_client = self.create_client(
            BodyPixWaveDetection, "/bodypix/detect_wave"
        )
        while not self.detect_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        # Set up the subscriber
        self.image_subscription = self.create_subscription(
            Image, self.listen_image_topic, self.image_callback, 10  # QoS profile
        )
        self.pcl_subscription = self.create_subscription(
            PointCloud2, self.listen_pcl_topic, self.pcl_callback, 10  # QoS profile
        )
        self.get_logger().info(
            f"Started listening on topic: {self.listen_image_topic}, {self.listen_pcl_topic}"
        )

    def detect(self, image):
        self.processing = True
        self.get_logger().info("Received image message")
        # Create a request for the service
        req = BodyPixWaveDetection.Request()
        req.image_raw = image
        # req.dataset = self.model
        req.confidence = 0.7

        # Call the service asynchronously
        future = self.detect_service_client.call_async(req)
        future.add_done_callback(self.detect_callback)

    def detect_callback(self, future):
        try:
            response = future.result()
            if response is not None:
                # Modify masks for demonstration purposes
                is_waving = response.wave_detected
                self.get_logger().info(f"Waving detection: {is_waving}")
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

    def pcl_callback(self, pcl):
        if self.processing:
            return

        # Start a new thread for detection to avoid blocking
        threading.Thread(target=self.detect, args=(pcl,)).start()


def main(args=None):
    print("Starting wave_relay node")
    # Check if command-line arguments are sufficient
    if len(sys.argv) < 2:
        print(
            "Usage: ros2 run lasr_vision_bodypix mask_relay.py <source_topic> [resnet50|mobilenet50|...]"
        )
        sys.exit(1)

    # Parse the command-line arguments
    listen_image_topic = "/image_raw"
    listen_pcl_topic = (
        "/xtion/depth_registered/points"  # not sure if this is the thing in ros2
    )
    if isinstance(sys.argv[1], list):
        listen_image_topic = sys.argv[1][0]
        listen_pcl_topic = sys.argv[1][1]

    rclpy.init(args=args)
    wave_relay_node = WaveRelay(listen_image_topic, listen_pcl_topic)
    wave_relay_node.get_logger().info("Mask relay node started")

    try:
        rclpy.spin(wave_relay_node)
    except KeyboardInterrupt:
        pass
    finally:
        wave_relay_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
