import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from lasr_vision_interfaces.srv import AddFace
import sys


def add_face(node: Node, name: str, num_images: int, image_topic: str):
    add_face_srv = node.create_client(AddFace, "/lasr_vision_reid/add_face")
    while not add_face_srv.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Service not available, waiting again...")

    images_collected = 0
    requests_in_flight = 0
    last_request_time_sec = 0.0
    collection_complete = False

    def handle_image(image: Image):
        nonlocal images_collected, requests_in_flight, last_request_time_sec, collection_complete

        if collection_complete:
            return

        # Do not queue more requests than needed.
        if images_collected + requests_in_flight >= num_images:
            return

        current_time_sec = node.get_clock().now().nanoseconds / 1e9
        if current_time_sec - last_request_time_sec < 1.0:
            return

        requests_in_flight += 1
        last_request_time_sec = current_time_sec

        def service_response_callback(future):
            nonlocal images_collected, requests_in_flight, collection_complete
            try:
                resp = future.result()
                if resp.success:
                    images_collected += 1
                    node.get_logger().info(
                        f"Added face '{name}' for image {images_collected}/{num_images}"
                    )
                    if images_collected >= num_images:
                        node.get_logger().info(
                            "Collected required number of images. Stopping collection."
                        )
                        collection_complete = True
                else:
                    node.get_logger().warning("Failed to add face for this image.")
            except Exception as e:
                node.get_logger().error(f"Service call failed: {e}")
            finally:
                requests_in_flight = max(0, requests_in_flight - 1)

        req = AddFace.Request()
        req.image_raw = image
        req.name = name

        # Use call_async() instead of call() to avoid blocking the event loop
        add_face_srv.call_async(req).add_done_callback(service_response_callback)

    image_sub = node.create_subscription(Image, image_topic, handle_image, 10)

    # Spin until collection is complete, checking in a loop to allow graceful exit
    while not collection_complete and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_subscription(image_sub)


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("lasr_vision_reid_add_face")

    node.declare_parameter("camera", "head_front_camera")
    camera = node.get_parameter("camera").value
    image_topic = f"/{camera}/rgb/image_raw"

    # camera = node.declare_parameter("~camera", "xtion").value
    name = node.declare_parameter("~name", "fadi").value  # originally jared
    num_images = node.declare_parameter("~num_images", 10).value
    image_topic = "image_raw"

    node.get_logger().info(f"Image topic: {image_topic}")

    node.declare_parameter("name", "fadi")
    name = node.get_parameter("name").value

    node.declare_parameter("num_images", 10)
    num_images = node.get_parameter("num_images").value

    add_face(node, name, num_images, image_topic)

    node.destroy_node()  # Added: Cleanup
    rclpy.shutdown()  # Added: Cleanup


if __name__ == "__main__":
    main()
