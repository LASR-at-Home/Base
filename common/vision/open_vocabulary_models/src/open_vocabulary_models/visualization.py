#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lasr_vision_interfaces.srv import OpenVocabDetect
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
import colorsys
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from cv_bridge import CvBridge
from rclpy.time import Time as ROS2Time
from rclpy.duration import Duration as ROS2Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class DetectionVisualizer(Node):
    def __init__(self):
        super().__init__('detection_visualizer')
        self.cli = self.create_client(OpenVocabDetect, 'open_vocab/detect')
        self.pub_markers = self.create_publisher(MarkerArray, '/detection_markers', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()

        self.cached_rgb = None
        self.cached_depth = None
        self.cached_camera_info = None
        self.cached_transform = None
        self.all_ready = False

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self._rgb_callback, qos)
        self.create_subscription(Image, '/head_front_camera/depth/image_raw', self._depth_callback, qos)
        self.create_subscription(CameraInfo, '/head_front_camera/rgb/camera_info', self._camera_info_callback, qos)
        self.create_timer(0.1, self._process_loop)

    def _rgb_callback(self, msg):
        first = self.cached_rgb is None
        self.cached_rgb = msg
        if first:
            self.get_logger().info("RGB received")
        self._try_get_transform()

    def _depth_callback(self, msg):
        first = self.cached_depth is None
        self.cached_depth = msg
        if first:
            self.get_logger().info("Depth received")
            self._check_all_ready()

    def _camera_info_callback(self, msg):
        if self.cached_camera_info is None:
            self.cached_camera_info = msg
            self.get_logger().info("CameraInfo received")
            self._check_all_ready()

    def _try_get_transform(self):
        if self.cached_rgb is None:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', self.cached_rgb.header.frame_id,
                ROS2Time(seconds=0), timeout=ROS2Duration(seconds=0.01)
            )
            first = self.cached_transform is None
            self.cached_transform = transform
            if first:
                self.get_logger().info("Transform received")
                self._check_all_ready()
        except Exception as e:
            if not hasattr(self, '_transform_error_logged'):
                self.get_logger().warn(f"Transform not available: {e}")
                self._transform_error_logged = True

    def _check_all_ready(self):
        if all([self.cached_rgb, self.cached_depth, self.cached_camera_info, self.cached_transform]):
            if not self.all_ready:
                self.get_logger().info("All data ready")
                self.all_ready = True

    def _process_loop(self):
        if not self.all_ready:
            return
        try:
            depth_image = self.bridge.imgmsg_to_cv2(self.cached_depth, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        req = OpenVocabDetect.Request()
        req.image = self.cached_rgb
        req.queries = ['can', 'bottle', 'table', 'person', 'wall', 'shelf', 'object', 'chair']
        req.box_threshold = 0.3
        req.text_threshold = 0.1
        future = self.cli.call_async(req)
        future.add_done_callback(lambda f: self._on_detection(f, depth_image))

    def _on_detection(self, future, depth_image):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'Detection failed: {e}')
            return

        K = self.cached_camera_info.k
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]
        camera_frame = self.cached_rgb.header.frame_id
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        for i, det in enumerate(resp.detections):
            x = int(np.clip(det.xywh[0], 0, depth_image.shape[1] - 1))
            y = int(np.clip(det.xywh[1], 0, depth_image.shape[0] - 1))
            depth = depth_image[y, x]

            if depth <= 0 or np.isnan(depth):
                continue

            point_cam = PointStamped()
            point_cam.header.frame_id = camera_frame
            point_cam.header.stamp = self.cached_rgb.header.stamp
            point_cam.point.x = float((x - cx) * depth / fx)
            point_cam.point.y = float((y - cy) * depth / fy)
            point_cam.point.z = float(depth)

            try:
                point_map = do_transform_point(point_cam, self.cached_transform)
            except Exception as e:
                self.get_logger().warn(f'Transform failed for {det.name}: {e}')
                continue

            hue = hash(det.name) % 360 / 360.0
            r, g, b = colorsys.hsv_to_rgb(hue, 0.8, det.confidence)

            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = stamp
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position = point_map.point
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.1
            marker.color.r, marker.color.g, marker.color.b = r, g, b
            marker.color.a = det.confidence

            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.id = 1000 + i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position = point_map.point
            text_marker.pose.position.z += 0.15
            text_marker.scale.z = 0.1
            text_marker.color.r = text_marker.color.g = text_marker.color.b = text_marker.color.a = 1.0
            text_marker.text = f'{det.name}\n{det.confidence:.2f}'

            markers.markers.extend([marker, text_marker])

        if markers.markers:
            self.pub_markers.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionVisualizer()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
