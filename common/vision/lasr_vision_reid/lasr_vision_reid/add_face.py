import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from lasr_vision_interfaces.srv import AddFace
import time
import sys


def add_face(node: Node, name: str, num_images: int, image_topic: str):
    add_face_srv = node.create_client(AddFace, "/lasr_vision_reid/add_face")
    while not add_face_srv.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Service not available, waiting again...")

    images_collected = 0
    last_processed_time = 0.0

    def handle_image(image: Image):
        nonlocal images_collected, last_processed_time
        current_time = time.time()
        if current_time - last_processed_time < 1.0:
            return

        if images_collected >= num_images:
            node.get_logger().info(
                "Collected required number of images. Unsubscribing."
            )
            node.destroy_subscription(image_sub)  # Changed: Use destroy_subscription
            return

        req = AddFace.Request()  # Changed: Use .Request
        req.image_raw = image
        req.name = name

        try:
            resp = add_face_srv.call(req)
            if resp.success:
                images_collected += 1
                last_processed_time = current_time
                node.get_logger().info(
                    f"Added face '{name}' for image {images_collected}/{num_images}"
                )
            else:
                node.get_logger().warning("Failed to add face for this image.")
        except (
            Exception
        ) as e:  # Changed: Generic exception instead of rospy.ServiceException
            node.get_logger().error(f"Service call failed: {e}")

    image_sub = node.create_subscription(
        Image, image_topic, handle_image, 10
    )  # Changed: Add QoS depth
    rclpy.spin(node)


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("lasr_vision_reid_add_face")

    node.declare_parameter("camera", "head_front_camera")
    camera = node.get_parameter("camera").value
    image_topic = f"/{camera}/rgb/image_raw"
    node.get_logger().info(f"Image topic: {image_topic}")

    # camera = node.declare_parameter("~camera", "xtion").value
    name = node.declare_parameter("~name", "fadi").value  # originally jared
    num_images = node.declare_parameter("~num_images", 10).value
    # image_topic = f"/{camera}/rgb/image_raw"

    node.declare_parameter("name", "fadi")
    name = node.get_parameter("name").value

    node.declare_parameter("num_images", 10)
    num_images = node.get_parameter("num_images").value

    add_face(node, name, num_images, image_topic)

    node.destroy_node()  # Added: Cleanup
    rclpy.shutdown()  # Added: Cleanup


if __name__ == "__main__":
    main()
