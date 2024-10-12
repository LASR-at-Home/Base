#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import lasr_vision_bodypix as bodypix
from lasr_vision_msgs.srv import (
    BodyPixMaskDetection,
    BodyPixKeypointDetection,
    DetectWave,
)
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
from cv2_pcl import pcl_to_img_msg
import numpy as np
import struct
from typing import Union

class BodyPixServiceNode(Node):
    def __init__(self):
        super().__init__("bodypix_service_node")

        # Initialize parameters
        self.declare_parameter('preload', [])
        preload_models = self.get_parameter('preload').get_parameter_value().string_array_value

        # Preload models if specified
        for model in preload_models:
            bodypix.load_model_cached(model)

        # Create service servers
        self.mask_service = self.create_service(
            BodyPixMaskDetection, '/bodypix/mask_detection', self.detect_masks)
        self.keypoint_service = self.create_service(
            BodyPixKeypointDetection, '/bodypix/keypoint_detection', self.detect_keypoints)
        self.detect_wave_service = self.create_service(
            DetectWave, '/bodypix/detect_wave', self.detect_wave)

        # Debug publisher for detect_wave
        self.debug_publisher = self.create_publisher(Image, "debug_waving", 1)

        self.get_logger().info("BodyPix service node started")

    def detect_masks(self, request, response):
        """Detects body part masks."""
        response = bodypix.detect_masks(request)
        return response

    def detect_keypoints(self, request, response):
        """Detects keypoints."""
        response = bodypix.detect_keypoints(request)
        return response

    def detect_wave(self, request, response):
        """Detects waving gesture."""
        try:
            # Convert PointCloud2 to image (if needed by your BodyPix model)
            bp_req = BodyPixKeypointDetection.Request()
            bp_req.image_raw = pcl_to_img_msg(request.pcl_msg)  # Assuming you have a pcl_to_img_msg function.
            bp_req.dataset = request.dataset
            bp_req.confidence = request.confidence

            # Detect keypoints using the BodyPix model
            detected_keypoints = bodypix.detect_keypoints(bp_req).keypoints
        except Exception as e:
            self.get_logger().error(f"Error detecting keypoints: {e}")
            return DetectWave.Response()

        # Analyze keypoints
        gesture_to_detect = None
        keypoint_info = {
            keypoint.keypoint_name: {"x": keypoint.x, "y": keypoint.y}
            for keypoint in detected_keypoints
        }

        if "leftShoulder" in keypoint_info and "leftWrist" in keypoint_info:
            if keypoint_info["leftWrist"]["y"] < keypoint_info["leftShoulder"]["y"]:
                gesture_to_detect = "raising_left_arm"

        if "rightShoulder" in keypoint_info and "rightWrist" in keypoint_info:
            if keypoint_info["rightWrist"]["y"] < keypoint_info["rightShoulder"]["y"]:
                gesture_to_detect = "raising_right_arm"

        if gesture_to_detect is not None:
            self.get_logger().info(f"Detected gesture: {gesture_to_detect}")

        # Convert PointCloud2 to XYZ array
        pcl_xyz = self.pointcloud2_to_xyz_array(request.pcl_msg)

        # Determine wave position
        try:
            wave_position = np.zeros(3)
            wave_point = keypoint_info.get(
                "leftShoulder" if gesture_to_detect == "raising_left_arm" else "rightShoulder"
            )

            # Take the average of points around the detected keypoint
            for i in range(-5, 5):
                for j in range(-5, 5):
                    if np.any(np.isnan(pcl_xyz[int(wave_point["y"]) + i][int(wave_point["x"]) + j])):
                        self.get_logger().warn("Nan point in pcl")
                        continue
                    wave_position += pcl_xyz[int(wave_point["y"]) + i][int(wave_point["x"]) + j]
            wave_position /= 100

            wave_position_msg = PointStamped(
                point=Point(*wave_position),
                header=Header(frame_id=request.pcl_msg.header.frame_id),
            )
            self.get_logger().info(f"Wave point: {wave_position_msg}")
        except Exception as e:
            self.get_logger().error(f"Error getting wave point: {e}")
            wave_position_msg = PointStamped()

        is_waving = gesture_to_detect is not None

        # Build response
        response.keypoints = detected_keypoints
        response.wave_detected = is_waving
        response.wave_position = wave_position_msg
        return response

    def pointcloud2_to_xyz_array(self, cloud_msg):
        """
        Converts a PointCloud2 message to a numpy array of shape (N, 3), with fields x, y, z.
        """
        points = []
        for point in self.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])
        return np.array(points)

    def read_points(self, cloud, field_names=None, skip_nans=True):
        """
        Generator for reading points from a PointCloud2 message. 
        Optionally filters by field_names and skips NaNs.
        """
        fmt = "fff"  # Format for x, y, z
        point_step = cloud.point_step
        for i in range(0, len(cloud.data), point_step):
            x, y, z = struct.unpack_from(fmt, cloud.data, i)
            if skip_nans and (x != x or y != y or z != z):  # Check for NaNs
                continue
            yield (x, y, z)


def main(args=None):
    rclpy.init(args=args)
    node = BodyPixServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
