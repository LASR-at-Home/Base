import rclpy
from rclpy.node import Node
import message_filters
import sys
from threading import Thread

from sensor_msgs.msg import Image, CameraInfo
from lasr_vision_interfaces.srv import Recognise


def relay_2d(node: Node, image_topic: str) -> None:

    recognise = node.create_client(Recognise, "/lasr_vision_reid/recognise/twod")
    while not recognise.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Service not available, waiting again...")
    node.get_logger().info("Service is ready!")

    def detect_cb(image: Image):
        def response_callback(future):
            try:
                response = future.result()
                node.get_logger().info(str(response))
            except Exception as e:
                node.get_logger().error(f"Service call failed: {e}")

        request = Recognise.Request(
            image_raw=image,
            confidence=0.5,
        )
        # Use async with threading - callback executes in spin thread where TF is updated
        recognise.call_async(request).add_done_callback(response_callback)

    image_sub = node.create_subscription(Image, image_topic, detect_cb, 10)


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("lasr_vision_reid_relay_2d")

    image_topic = "image_raw"
    node.get_logger().info(f"Image topic: {image_topic}")

    relay_2d(
        node=node,
        image_topic=image_topic,
    )

    # Run spin in a separate thread so service calls don't block the event loop
    spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Keep the main thread alive
    try:
        while True:
            spin_thread.join(timeout=1.0)
            if not spin_thread.is_alive():
                break
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
