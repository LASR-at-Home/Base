import rclpy
from rclpy.node import Node
import message_filters
import sys

from sensor_msgs.msg import Image, CameraInfo
from lasr_vision_interfaces.srv import Recognise3D


def relay_3d(
    node: Node, image_topic: str, depth_topic: str, depth_camera_info_topic: str
) -> None:

    recognise = node.create_client(Recognise3D, "/lasr_vision_reid/recognise")
    while not recognise.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Service not available, waiting again...")
    node.get_logger().info("Service is ready!")

    def detect_cb(image: Image, depth_image: Image, depth_camera_info: CameraInfo):
        request = Recognise3D.Request(
            image_raw=image,
            depth_image=depth_image,
            depth_camera_info=depth_camera_info,
            threshold=0.5,
            target_frame="map",
        )
        response = recognise.call_async(request)
        node.get_logger().info(response)

    image_sub = message_filters.Subscriber(node, Image, image_topic)
    depth_sub = message_filters.Subscriber(node, Image, depth_topic)
    depth_camera_info_sub = message_filters.Subscriber(
        node, CameraInfo, depth_camera_info_topic
    )
    ts = message_filters.ApproximateTimeSynchronizer(
        [image_sub, depth_sub, depth_camera_info_sub], 10, 2.0
    )
    ts.registerCallback(detect_cb)


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("lasr_vision_reid_relay")

    node.declare_parameter("camera", "xtion")
    camera = node.get_parameter("camera").value
    image_topic = f"/{camera}/rgb/image_raw"
    depth_topic = f"/{camera}/depth_registered/image_raw"
    depth_camera_info_topic = f"/{camera}/depth_registered/camera_info"

    relay_3d(node=node, image_topic=image_topic, depth_topic=depth_topic, depth_camera_info_topic=depth_camera_info_topic)

    rclpy.spin(node)
    node.destroy_node()  # Added: Cleanup
    rclpy.shutdown()  # Added: Cleanup


if __name__ == "__main__":
    main()
