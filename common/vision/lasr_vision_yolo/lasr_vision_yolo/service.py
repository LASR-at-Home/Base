#!/usr/bin/env python3

import os
from ament_index_python import packages

from typing import Dict, Union, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

import ultralytics
import torch
import numpy as np
import cv2

from lasr_vision_interfaces.srv import (
    YoloDetection,
    YoloDetection3D,
    YoloPoseDetection,
    YoloPoseDetection3D,
)
from lasr_vision_interfaces.msg import (
    Detection,
    Detection3D,
    Keypoint,
    Keypoint3D,
    KeypointList,
    Keypoint3DList,
)

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros as tf
from tf2_ros.buffer import Buffer
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point

KEYPOINT_MAPPING: Dict[int, str] = {
    0: "nose",
    1: "left_eye",
    2: "right_eye",
    3: "left_ear",
    4: "right_ear",
    5: "left_shoulder",
    6: "right_shoulder",
    7: "left_elbow",
    8: "right_elbow",
    9: "left_wrist",
    10: "right_wrist",
    11: "left_hip",
    12: "right_hip",
    13: "left_knee",
    14: "right_knee",
    15: "left_ankle",
    16: "right_ankle",
}

SKELETON_CONNECTIONS: List[Tuple[int, int]] = [
    (5, 6),  # shoulders
    (5, 7),
    (7, 9),  # left arm
    (6, 8),
    (8, 10),  # right arm
    (11, 12),  # hips
    (5, 11),
    (6, 12),  # torso
    (11, 13),
    (13, 15),  # left leg
    (12, 14),
    (14, 16),  # right leg
    (0, 1),
    (0, 2),  # nose to eyes
    (1, 3),
    (2, 4),  # eyes to ears
    (0, 5),
    (0, 6),  # nose to shoulders
]


class YOLOServiceNode:

    _cache: Dict[str, ultralytics.YOLO]
    _image_publishers: Dict[str, rclpy.node.Publisher]
    _marker_publishers: Dict[str, rclpy.node.Publisher]
    _bridge: CvBridge
    _tf_buffer: Buffer
    _tf_listener: tf.TransformListener

    def __init__(self, node: Node):
        self.node = node
        self._cache = {}
        self.node.declare_parameter(
            "~device", "cuda:0" if torch.cuda.is_available() else "cpu"
        )
        self._device = self.node.get_parameter("~device").value

        self.node.declare_parameter("~preload", ["yolo11n-seg.pt"])
        self.preload_param_list = self.node.get_parameter("~preload").value
        for model in self.preload_param_list:
            self._maybe_load_model(model)

        self._image_publishers = {}
        self._marker_publishers = {}
        self._bridge = CvBridge()

        self._image_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10))
        self._tf_listener = tf.TransformListener(self._tf_buffer, self.node)

        self.node.create_service(YoloDetection, "/yolo/detect", self._detect)
        self.node.create_service(YoloDetection3D, "/yolo/detect3d", self._detect3d)
        self.node.create_service(
            YoloPoseDetection, "/yolo/detect_pose", self._detect_keypoints
        )
        self.node.create_service(
            YoloPoseDetection3D, "/yolo/detect3d_pose", self._detect_keypoints3d
        )
        self.node.get_logger().info("YOLO service started")

    def _detect(
        self, req: YoloDetection.Request, res: YoloDetection.Response
    ) -> YoloDetection.Response:
        response = YoloDetection.Response()

        self.node.get_logger().info("Decoding")
        cv_im = self._bridge.imgmsg_to_cv2(req.image_raw, desired_encoding="bgr8")
        results = self._yolo(
            cv_im, req.model, req.confidence, [cls for cls in req.filter]
        )

        has_masks = results.masks is not None

        for result in results:
            detection = Detection()
            detection.name = result.names[result.boxes.cls.int().item()]
            detection.confidence = result.boxes.conf.item()
            detection.xywh = (
                result.boxes.xywh.round().int().squeeze().cpu().numpy().tolist()
            )

            if has_masks:
                detection.xyseg = (
                    np.array(result.masks.xy).flatten().round().astype(int).tolist()
                )

            response.detected_objects.append(detection)

        self._publish_results(req, results, response)

        return response

    def _project_mask_to_3d(
        self,
        mask_xy: np.ndarray,
        depth_im: np.ndarray,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
    ) -> tuple:
        """Project segmentation mask to 3D using depth image and camera intrinsics.

        Uses standard pinhole camera model:
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        Assumes depth_im is in millimeters.
        Returns: (x, y, z) median point in camera frame, in meters.
        """
        contours = np.array(mask_xy, dtype=np.int32).reshape(-1, 2)
        mask = np.zeros(depth_im.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [contours], color=255)
        roi = cv2.bitwise_and(depth_im, depth_im, mask=mask)

        v, u = np.where(roi)
        z = depth_im[v, u] / 1000.0  # convert mm to meters
        valid = z > 0
        z = z[valid]
        u = u[valid]
        v = v[valid]

        x = z * (u - cx) / fx
        y = z * (v - cy) / fy
        points = np.stack((x, y, z), axis=1)

        return np.median(points, axis=0)

    def _lookup_transform(self, target_frame, source_frame, stamp):
        try:
            return self._tf_buffer.lookup_transform(
                target_frame, source_frame, stamp, Duration(seconds=1.0)
            )
        except Exception as e:
            self.node.get_logger().debug(
                f"TF {target_frame}<-{source_frame} at image stamp failed ({e}); using latest"
            )
        try:
            return self._tf_buffer.lookup_transform(
                target_frame, source_frame, Time(), Duration(seconds=1.0)
            )
        except Exception as e:
            self.node.get_logger().error(
                f"TF {target_frame}<-{source_frame} lookup failed: {e}"
            )
            raise

    def _detect3d(
        self, req: YoloDetection3D.Request, res: YoloDetection3D.Response
    ) -> YoloDetection3D.Response:
        response = YoloDetection3D.Response()

        cv_im = self._bridge.imgmsg_to_cv2(req.image_raw, desired_encoding="bgr8")
        results = self._yolo(
            cv_im, req.model, req.confidence, [cls for cls in req.filter]
        )
        depth_im = self._bridge.imgmsg_to_cv2(
            req.depth_image, desired_encoding="passthrough"
        )
        K = req.depth_camera_info.k
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]

        target_frame = req.target_frame or "map"

        transform = None
        has_detections = len(results.boxes) > 0 if hasattr(results, "boxes") else False
        if has_detections:
            transform = self._lookup_transform(
                target_frame,
                req.depth_image.header.frame_id,
                req.depth_image.header.stamp,
            )

        for result in results:
            detection = Detection3D()
            detection.name = result.names[result.boxes.cls.int().item()]
            detection.confidence = result.boxes.conf.item()
            bbox = result.boxes.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = bbox
            detection.xywh = [
                int(round(x1)),
                int(round(y1)),
                int(round(x2 - x1)),
                int(round(y2 - y1)),
            ]

            has_mask = result.masks is not None
            if has_mask:
                detection.xyseg = (
                    np.array(result.masks.xy).flatten().round().astype(int).tolist()
                )
                x, y, z = self._project_mask_to_3d(
                    detection.xyseg, depth_im, fx, fy, cx, cy
                )

                point = Point(x=float(x), y=float(y), z=float(z))
                point_stamped = PointStamped()
                point_stamped.header = req.depth_image.header
                point_stamped.point = point
                if transform is not None:
                    point_stamped_transformed = do_transform_point(
                        point_stamped, transform
                    )
                    detection.point = point_stamped_transformed.point
                else:
                    detection.point = point

            else:
                self.node.get_logger().warn(
                    "3D Estimation is not implemented when masks aren't available."
                )

            response.detected_objects.append(detection)

        self._publish_results(req, results, response)

        return response

    def _detect_keypoints(
        self, req: YoloPoseDetection.Request, res: YoloPoseDetection.Response
    ) -> YoloPoseDetection.Response:
        response = YoloPoseDetection.Response()

        cv_im = self._bridge.imgmsg_to_cv2(req.image_raw, desired_encoding="bgr8")
        results = self._yolo(cv_im, req.model, req.confidence, [])

        for result in results:
            keypoints = KeypointList()
            for idx, name in KEYPOINT_MAPPING.items():
                x = result.keypoints.xy.squeeze()[idx, 0].round().int().item()
                y = result.keypoints.xy.squeeze()[idx, 1].round().int().item()
                conf = result.keypoints.conf.squeeze()[idx].item()
                if conf > 0.0:
                    keypoints.keypoints.append(Keypoint(keypoint_name=name, x=x, y=y))
            response.detections.append(keypoints)

        self._publish_results(req, results, response)

        return response

    def _detect_keypoints3d(
        self, req: YoloPoseDetection3D.Request, res: YoloPoseDetection3D.Response
    ) -> YoloPoseDetection3D.Response:
        response = YoloPoseDetection3D.Response()

        cv_im = self._bridge.imgmsg_to_cv2(req.image_raw, desired_encoding="bgr8")
        results = self._yolo(cv_im, req.model, req.confidence, [])
        depth_im = self._bridge.imgmsg_to_cv2(
            req.depth_image, desired_encoding="passthrough"
        )
        K = req.depth_camera_info.k
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]

        target_frame = req.target_frame or "map"

        transform = None
        has_detections = len(results.boxes) > 0 if hasattr(results, "boxes") else False
        if has_detections:
            transform = self._lookup_transform(
                target_frame,
                req.depth_image.header.frame_id,
                req.depth_image.header.stamp,
            )

        for result in results:
            keypoints = Keypoint3DList()
            for idx, name in KEYPOINT_MAPPING.items():
                h, w = 480, 640
                u = result.keypoints.xy.squeeze()[idx, 0].round().int().item()
                u = min(u, w - 1)
                v = result.keypoints.xy.squeeze()[idx, 1].round().int().item()
                v = min(v, h - 1)

                conf = result.keypoints.conf.squeeze()[idx].item()
                if conf > 0.0:
                    z = depth_im[v, u] / 1000.0  # convert mm to meters
                    x = z * (u - cx) / fx
                    y = z * (v - cy) / fy
                    if np.isnan(x) or np.isnan(y) or np.isnan(z):
                        continue

                    point = Point(x=float(x), y=float(y), z=float(z))
                    point_stamped = PointStamped()
                    point_stamped.header = req.depth_image.header
                    point_stamped.point = point
                    if transform is not None:
                        point_stamped_transformed = do_transform_point(
                            point_stamped, transform
                        )
                        point = point_stamped_transformed.point

                    keypoints.keypoints.append(
                        Keypoint3D(keypoint_name=name, point=point)
                    )
            response.detections.append(keypoints)

        self._publish_results(req, results, response)

        return response

    def _maybe_load_model(self, model_name: str) -> ultralytics.YOLO:
        if model_name in self._cache:
            return self._cache[model_name]

        model = self._cache[model_name] = ultralytics.YOLO(model_name).to(self._device)

        self.node.get_logger().info(f"Loaded {model_name} model on {self._device}")
        return model

    def _publish_results(
        self,
        req: Union[YoloDetection.Request, YoloDetection3D.Request],
        results: ultralytics.engine.results.Results,
        response: Union[YoloDetection.Response, YoloDetection3D.Response],
    ) -> None:

        if req.model in self._image_publishers:
            image_publisher = self._image_publishers[req.model]
        else:
            image_publisher = self._image_publishers[req.model] = (
                self.node.create_publisher(
                    Image,
                    f"/yolo/detect/{req.model}".replace("-", "_").replace(".", "_"),
                    self._image_qos,
                )
            )

        image_msg = self._bridge.cv2_to_imgmsg(results.plot(), encoding="bgr8")
        image_msg.header.stamp = req.image_raw.header.stamp
        image_msg.header.frame_id = req.image_raw.header.frame_id
        image_publisher.publish(image_msg)

        if isinstance(response, YoloDetection3D.Response):

            if req.model in self._marker_publishers:
                marker_publisher = self._marker_publishers[req.model]
            else:
                marker_publisher = self._marker_publishers[req.model] = (
                    self.node.create_publisher(
                        Marker,
                        f"/yolo/detect3d/{req.model}".replace("-", "_").replace(
                            ".", "_"
                        ),
                        10,
                    )
                )

            for i, detection in enumerate(response.detected_objects):

                marker = Marker()
                target_frame = req.target_frame or "map"
                marker.header.frame_id = target_frame
                marker.header.stamp = req.depth_image.header.stamp
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose.position = detection.point

                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1

                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

                marker_publisher.publish(marker)

        elif isinstance(response, YoloPoseDetection3D.Response):
            if req.model in self._marker_publishers:
                marker_publisher = self._marker_publishers[req.model]
            else:
                marker_publisher = self._marker_publishers[req.model] = (
                    self.node.create_publisher(
                        MarkerArray,
                        f"/yolo/pose3d/{req.model}".replace("-", "_").replace(".", "_"),
                        10,
                    )
                )

            marker_array = MarkerArray()
            marker_id = 0

            for detection_idx, keypoint_list in enumerate(response.detections):
                points = [kp.point for kp in keypoint_list.keypoints]

                # Add spheres for each keypoint
                for i, pt in enumerate(points):
                    marker = Marker()
                    marker.header.frame_id = (
                        req.target_frame or req.depth_image.header.frame_id
                    )
                    marker.header.stamp = self.node.get_clock().now().to_msg()
                    marker.id = marker_id
                    marker.ns = f"person_{detection_idx}"
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.pose.position = pt
                    marker.scale.x = 0.05
                    marker.scale.y = 0.05
                    marker.scale.z = 0.05
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 1.0
                    marker_array.markers.append(marker)
                    marker_id += 1

                # Add line segments to represent skeleton
                for a, b in SKELETON_CONNECTIONS:
                    if a < len(points) and b < len(points):
                        line_marker = Marker()
                        line_marker.header.frame_id = (
                            req.target_frame or req.depth_image.header.frame_id
                        )
                        line_marker.header.stamp = self.node.get_clock().now().to_msg()
                        line_marker.id = marker_id
                        line_marker.ns = f"person_{detection_idx}_lines"
                        line_marker.type = Marker.LINE_LIST
                        line_marker.action = Marker.ADD
                        line_marker.scale.x = 0.02
                        line_marker.color.r = 0.0
                        line_marker.color.g = 0.0
                        line_marker.color.b = 1.0
                        line_marker.color.a = 1.0
                        line_marker.points.append(points[a])
                        line_marker.points.append(points[b])
                        marker_array.markers.append(line_marker)
                        marker_id += 1

            marker_publisher.publish(marker_array)

    def _yolo(
        self, img, model: str, conf: float, filter: List[str]
    ) -> ultralytics.engine.results.Results:
        yolo = self._maybe_load_model(model)
        filter_idx = (
            [{v: k for k, v in yolo.names.items()}[cls] for cls in filter]
            if filter
            else None
        )
        results = yolo(img, conf=conf, classes=filter_idx, verbose=False)[0]
        return results


def main(args=None):
    rclpy.init(args=args)

    node = Node("yolo_service")
    YOLOServiceNode(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
