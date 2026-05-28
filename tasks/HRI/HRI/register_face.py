#!/usr/bin/env python3
"""
Script to register a face with the ReID service.
Run this before running introduce_test to register guests.

Usage:
  1. Start the ReID service: ros2 run lasr_vision_reid service
  2. Start your webcam: ros2 run usb_cam usb_cam_node_exe
  3. Run this script: python3 register_face.py
"""

import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import Image
from lasr_vision_interfaces.srv import AddFace


class RegisterFace(Node):
    def __init__(self):
        super().__init__("register_face_node")

        self._client = self.create_client(AddFace, "/lasr_vision_reid/add_face")
        self.get_logger().info("Waiting for ReID add_face service...")
        if not self._client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("ReID add_face service not available.")
            return
        self.get_logger().info("ReID add_face service available.")

    def register(
        self, guest_id: str, image_topic: str = "/head_front_camera/rgb/image_raw"
    ):
        self.get_logger().info(
            f"Waiting for image to register '{guest_id}'... look at the camera!"
        )

        success, image = wait_for_message(Image, self, image_topic, time_to_wait=20.0)
        if not success:
            self.get_logger().error("No image received.")
            return False

        self.get_logger().info(f"Got image — registering '{guest_id}'...")

        request = AddFace.Request()
        request.image_raw = image
        request.name = guest_id

        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response is None:
            self.get_logger().error("AddFace service call failed.")
            return False

        self.get_logger().info(f"Successfully registered '{guest_id}'!")
        return True


def main():
    rclpy.init()
    node = RegisterFace()

    # Register host
    input("Press Enter when 'Sophie' (host) is looking at the camera...")
    node.register("host")

    # Register guest1
    input("Press Enter when 'John' (guest1) is looking at the camera...")
    node.register("guest1")

    node.get_logger().info("All guests registered! You can now run introduce_test.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
