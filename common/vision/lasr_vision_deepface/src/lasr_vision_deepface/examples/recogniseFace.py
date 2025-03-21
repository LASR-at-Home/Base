#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from lasr_vision_interfaces.srv import Recognise


class FaceRecognitionClient(Node):
    def __init__(self, listen_topic, dataset):
        super().__init__("face_recognition_client")

        # ROS2 Subscription
        self.subscription = self.create_subscription(
            Image, listen_topic, self.detect, 10
        )
        self.client = self.create_client(Recognise, "/recognise")

        # Store dataset name
        self.dataset = dataset
        self.people_in_frame = {}

    def detect(self, image):
        """Handles image detection and calls the Recognise service."""
        self.get_logger().info("Received image message")

        req = Recognise.Request()
        req.image_raw = image
        req.dataset = self.dataset
        req.confidence = 0.4

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            for detection in response.detections:
                self.people_in_frame[detection.name] = self.get_clock().now()
            self.get_logger().info("Service succeeded!")
        else:
            self.get_logger().error("Service call failed!")

    def image_callback(self, image):
        """Callback to process images and track people in the frame."""
        prev_people_in_frame = list(self.people_in_frame.keys())

        # Remove old detections
        self.detect(image)
        for person in list(self.people_in_frame.keys()):
            if self.get_clock().now() - self.people_in_frame[
                person
            ] > rclpy.duration.Duration(seconds=10):
                del self.people_in_frame[person]

        # Trigger greet() if people in frame change
        if (
            list(self.people_in_frame.keys()) != prev_people_in_frame
            and len(self.people_in_frame) > 0
        ) or (len(prev_people_in_frame) == 0 and len(self.people_in_frame) > 0):
            self.greet()

    def greet(self):
        """Example function that triggers when a new person is detected."""
        self.get_logger().info("Greeting a detected person!")


def main(args=None):
    """Main function to start the ROS2 node."""
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print(
            "Usage: ros2 run lasr_vision_deepface recognise_face <source_topic> <dataset>"
        )
        sys.exit(1)

    listen_topic = sys.argv[1]
    dataset = sys.argv[2]

    node = FaceRecognitionClient(listen_topic, dataset)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
