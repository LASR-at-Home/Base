import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from lasr_vision_msgs.srv import LangSam


class LangSamExample(Node):
    def __init__(self):
        super().__init__("lang_sam_example")

        self.camera_image = None

        self.subscription = self.create_subscription(
            Image, "/xtion/rgb/image_raw", self.image_callback, 1
        )

        self.client = self.create_client(LangSam, "/lasr_vision/lang_sam")

    def image_callback(self, image_msg):
        self.camera_image = image_msg

    def run(self):
        while rclpy.ok():
            if not self.client.wait_for_service(timeout_sec=0.2):
                self.get_logger().info("Waiting for /lasr_vision/lang_sam service...")
                rclpy.spin_once(self, timeout_sec=0.0)
                continue

            if self.camera_image is not None:
                req = LangSam.Request()
                req.image_raw = self.camera_image
                req.prompt = "bag"

                future = self.client.call_async(req)
                rclpy.spin_until_future_complete(self, future)

                if future.result() is not None:
                    resp = future.result()
                    print(resp.detections)

            rclpy.spin_once(self, timeout_sec=0.0)


def main(args=None):
    rclpy.init(args=args)

    node = LangSamExample()
    node.get_logger().info("Starting LangSAM example node")
    node.get_logger().info("Waiting for /lasr_vision/lang_sam service...")
    while not node.client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("/lasr_vision/lang_sam service is not available yet...")

    node.get_logger().info("/lasr_vision/lang_sam service is available")
    node.get_logger().info("Starting LangSAM example...")
    node.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()