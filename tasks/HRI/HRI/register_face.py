#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from lasr_vision_interfaces.srv import AddFace
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import ReliabilityPolicy
 
class RegisterFace(Node):
    def __init__(self):
        super().__init__("register_face_node")
        self._image = None
 
        self._client = self.create_client(AddFace, "/lasr_vision_reid/add_face")
        self.get_logger().info("Waiting for ReID add_face service...")
        if not self._client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("ReID add_face service not available.")
            return
        self.get_logger().info("ReID add_face service available.")
 
    def _image_cb(self, msg: Image):
        self._image = msg
 
    def register(self, guest_id: str, image_topic: str = "/head_front_camera/rgb/image_raw"):
        self._image = None
 
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history = HistoryPolicy.KEEP_LAST)
        sub = self.create_subscription(Image, image_topic, self._image_cb, qos)
 
        self.get_logger().info(f"Waiting for image to register '{guest_id}'...")
 
        # Spin until we get an image
        while self._image is None:
            rclpy.spin_once(self, timeout_sec=0.1)
 
        self.get_logger().info(f"Got image — registering '{guest_id}'...")
        self.destroy_subscription(sub)
 
        request = AddFace.Request()
        request.image_raw = self._image
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
 
    input("Press Enter when 'John' (guest1) is looking at the camera...")
    node.register("guest1")
 
    node.get_logger().info("All guests registered!")
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()