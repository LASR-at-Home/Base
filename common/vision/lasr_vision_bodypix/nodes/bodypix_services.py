#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import lasr_vision_bodypix as bodypix
from lasr_vision_interfaces.srv import (
    BodyPixMaskDetection,
    BodyPixKeypointDetection,
    BodyPixWaveDetection,
)
from sensor_msgs.msg import Image

# import ros2_numpy as rnp
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
import numpy as np
from cv2_pcl import pcl_to_img_msg


class BodyPixServiceNode(Node):
    def __init__(self):
        super().__init__("bodypix_service_node")

        bodypix.load_model_cached(
            "resnet50"
        )  # by calling this the model will be loaded and cached

        # Create service servers
        self.mask_service = self.create_service(
            BodyPixMaskDetection, "/bodypix/mask_detection", self.detect_masks
        )
        self.keypoint_service = self.create_service(
            BodyPixKeypointDetection,
            "/bodypix/keypoint_detection",
            self.detect_keypoints,
        )
        self.get_logger().info("Keypoint detection service registered.")
        # self.detect_wave_service = self.create_service(
        #     BodyPixWaveDetection, "/bodypix/detect_wave", self.detect_wave
        # )

        # Debug publisher for detect_wave
        self.debug_publisher = self.create_publisher(Image, "debug_bodypix", 1)

        self.get_logger().info("BodyPix service node started")

    def detect_masks(self, request, response):
        """Handles mask detection request."""
        response = bodypix.detect_masks(
            request, debug_publisher=self.debug_publisher, logger=self.get_logger()
        )
        return response

    def detect_keypoints(self, request, response):
        """Handles keypoint detection request."""
        response = bodypix.detect_keypoints(
            request, debug_publisher=self.debug_publisher, logger=self.get_logger()
        )
        return response

    # def detect_wave(self, request, response):
    #     """Detects a waving gesture."""
    #     try:
    #         # Prepare request for keypoint detection
    #         bp_req = BodyPixKeypointDetection.Request()
    #         bp_req.image_raw = pcl_to_img_msg(request.pcl_msg)
    #         # bp_req.image_raw = request.image_raw
    #         bp_req.confidence = request.confidence

    #         # Call BodyPix keypoint detection
    #         detected_keypoints = bodypix.detect_keypoints(
    #             bp_req, debug_publisher=self.debug_publisher, logger=self.get_logger()
    #         ).keypoints
    #     except Exception as e:
    #         self.get_logger().error(f"Error detecting keypoints: {e}")
    #         return BodyPixWaveDetection.Response()

    #     gesture_to_detect = None
    #     keypoint_info = {
    #         keypoint.keypoint_name: {"x": keypoint.x, "y": keypoint.y}
    #         for keypoint in detected_keypoints
    #     }

    #     if "leftShoulder" in keypoint_info and "leftWrist" in keypoint_info:
    #         if keypoint_info["leftWrist"]["y"] < keypoint_info["leftShoulder"]["y"]:
    #             gesture_to_detect = "raising_left_arm"
    #     if "rightShoulder" in keypoint_info and "rightWrist" in keypoint_info:
    #         if keypoint_info["rightWrist"]["y"] < keypoint_info["rightShoulder"]["y"]:
    #             gesture_to_detect = "raising_right_arm"

    #     if gesture_to_detect is not None:
    #         self.get_logger().info(f"Detected gesture: {gesture_to_detect}")

    #     # Process wave point in point cloud
    #     wave_point = keypoint_info.get(
    #         "leftShoulder"
    #         if gesture_to_detect == "raising_left_arm"
    #         else "rightShoulder"
    #     )
    #     pcl_xyz = rnp.point_cloud2.pointcloud2_to_xyz_array(
    #         request.pcl_msg, remove_nans=False
    #     )

    #     try:
    #         wave_position = np.zeros(3)
    #         for i in range(-5, 5):
    #             for j in range(-5, 5):
    #                 if np.any(
    #                     np.isnan(
    #                         pcl_xyz[int(wave_point["y"]) + i][int(wave_point["x"]) + j]
    #                     )
    #                 ):
    #                     self.get_logger().warn("NaN point in PCL")
    #                     continue
    #                 wave_position += pcl_xyz[int(wave_point["y"]) + i][
    #                     int(wave_point["x"]) + j
    #                 ]
    #         wave_position /= 100
    #         wave_position_msg = PointStamped(
    #             point=Point(*wave_position),
    #             header=Header(frame_id=request.pcl_msg.header.frame_id),
    #         )
    #         self.get_logger().info(f"Wave point: {wave_position_msg}")
    #     except Exception as e:
    #         self.get_logger().error(f"Error getting wave point: {e}")
    #         wave_position_msg = PointStamped()

    #     is_waving = gesture_to_detect is not None

    #     # Build response
    #     response.keypoints = detected_keypoints
    #     response.wave_detected = is_waving
    #     response.wave_position = wave_position_msg
    #     return response


def main(args=None):
    rclpy.init(args=args)
    node = BodyPixServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
