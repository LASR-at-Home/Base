import rclpy
from rclpy.node import Node
import message_filters
import sys
from threading import Thread

from sensor_msgs.msg import Image, CameraInfo
from lasr_vision_interfaces.srv import Recognise3D


def relay_3d(
    node: Node, image_topic: str, depth_topic: str, depth_camera_info_topic: str
) -> None:

    recognise = node.create_client(Recognise3D, "/lasr_vision_reid/recognise/threed")
    while not recognise.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Service not available, waiting again...")
    node.get_logger().info("Service is ready!")

    cam_info = None

    def cache_camera_info(msg: CameraInfo) -> None:
        nonlocal cam_info
        if cam_info is None:
            cam_info = msg

    node.create_subscription(CameraInfo, depth_camera_info_topic, cache_camera_info, 10)

    def detect_cb(image: Image, depth_image: Image):
        if cam_info is None:
            node.get_logger().warn("Camera info not yet available")
            return

        def response_callback(future):
            try:
                response = future.result()
                node.get_logger().info(str(response))
            except Exception as e:
                node.get_logger().error(f"Service call failed: {e}")

        request = Recognise3D.Request(
            image_raw=image,
            depth_image=depth_image,
            depth_camera_info=cam_info,
            threshold=0.5,
            target_frame="map",
        )
        # Use async with threading - callback executes in spin thread where TF is updated
        recognise.call_async(request).add_done_callback(response_callback)

    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

    camera_qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
    )

    image_sub = message_filters.Subscriber(
        node, Image, image_topic, qos_profile=camera_qos
    )
    depth_sub = message_filters.Subscriber(
        node, Image, depth_topic, qos_profile=camera_qos
    )
    ts = message_filters.ApproximateTimeSynchronizer(
        [image_sub, depth_sub], queue_size=30, slop=0.1
    )
    ts.registerCallback(detect_cb)


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("lasr_vision_reid_relay_3d")

    node.declare_parameter("camera", "head_front_camera")
    camera = node.get_parameter("camera").value
    image_topic = f"/{camera}/rgb/image_raw"
    depth_topic = f"/{camera}/depth/image_raw"
    depth_camera_info_topic = f"/{camera}/depth/camera_info"
    node.get_logger().info(f"Image topic: {image_topic}")
    node.get_logger().info(f"Depth topic: {depth_topic}")
    node.get_logger().info(f"Depth camera info topic: {depth_camera_info_topic}")

    relay_3d(
        node=node,
        image_topic=image_topic,
        depth_topic=depth_topic,
        depth_camera_info_topic=depth_camera_info_topic,
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
