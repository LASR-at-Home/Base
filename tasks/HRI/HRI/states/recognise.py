from typing import List, Dict, Optional


import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
import numpy as np
import cv2


from smach import UserData
from smach_ros import RosState
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

import message_filters
from lasr_vision_interfaces.msg import Detection3D
from lasr_vision_interfaces.srv import Recognise3D, YoloDetection3D

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import ReliabilityPolicy

qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history = HistoryPolicy.KEEP_LAST)


class Recognise(RosState):

    _rgb_image_topic: str
    _depth_image_topic: str
    _depth_info_topic: str
    _bridge: CvBridge
    _can_detect_second_guest: bool = False

    def __init__(self, node: Node, can_detect_second_guest: bool = False):
        super().__init__(
            node,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data", "guest_seat_point", "seated_guest_locs"],
            output_keys=["named_guest_detection", "guest_data"],
        )
        self._rgb_image_topic = "/head_front_camera/rgb/image_raw"
        self._depth_image_topic = "/head_front_camera/depth/image_raw"
        self._depth_info_topic = "/head_front_camera/depth/camera_info"

        self._can_detect_second_guest = can_detect_second_guest
        self._bridge = CvBridge()

    def _handle_no_detections(self, guest_data: Dict) -> Optional[Detection3D]:
        """
        Fallback when no guest is recognised. Iterates through guests and returns
        the first one that hasn't been detected yet in this sweep.
        """
        for guest_id, data in guest_data.items():
            if data["seating_detection"]:
                continue
            if guest_id == "guest2" and not self._can_detect_second_guest:
                continue
            detection = Detection3D()
            detection.name = guest_id
            guest_data[guest_id]["seating_detection"] = True
            return detection
        return None

    def _crop_image(
        self, person_detections: List[Detection3D], rgb_image: Image
    ) -> Image:
        image_width, image_height = rgb_image.width, rgb_image.height
        centre_x = image_width // 2
        centre_y = image_height // 2
        closest_distance = float("inf")
        closest_detection = None

        for detection in person_detections:
            x, y, w, h = detection.xywh
            bbox_centre_x = x + w // 2
            bbox_centre_y = y + h // 2
            distance_to_centre = np.abs(bbox_centre_x - centre_x) + np.abs(
                bbox_centre_y - centre_y
            )
            if distance_to_centre < closest_distance:
                closest_distance = distance_to_centre
                closest_detection = detection

        assert closest_detection is not None, "No person detection found to crop."

        # Use bounding box instead of segmentation mask
        x, y, w, h = closest_detection.xywh
        rgb_image_raw = self._bridge.imgmsg_to_cv2(rgb_image, desired_encoding="rgb8")
        cropped = rgb_image_raw[y : y + h, x : x + w]
        return self._bridge.cv2_to_imgmsg(cropped, encoding="rgb8")

    def execute(self, userdata: UserData) -> str:
 
        # --- ROS 2: create clients instead of ServiceProxy ---

        recognise_client = self.node.create_client(

            Recognise3D, "/lasr_vision_reid/recognise/threed"

        )

        yolo_client = self.node.create_client(YoloDetection3D, "/yolo/detect3d")
    
        if not recognise_client.wait_for_service(timeout_sec=5.0):

            self.node.get_logger().error(

                "Recognise3D service /lasr_vision_reid/recognise/threed not available"

            )

            return "failed"

        if not yolo_client.wait_for_service(timeout_sec=5.0):

            self.node.get_logger().error(

                "YoloDetection3D service /yolo/detect3d not available"

            )

            return "failed"
    
        # Reset images on each execute() call to ensure fresh data

        self._rgb_image = None

        self._depth_image = None

        self._depth_camera_info = None
        
        def rgb_cb(msg):

            self._rgb_image = msg
    
        def depth_cb(msg):

            self._depth_image = msg
    
        def info_cb(msg):

            self._depth_camera_info = msg
    
        rgb_sub = self.node.create_subscription(

            Image, self._rgb_image_topic, rgb_cb, qos

        )

        depth_sub = self.node.create_subscription(

            Image, self._depth_image_topic, depth_cb, qos

        )

        info_sub = self.node.create_subscription(

            CameraInfo, self._depth_info_topic, info_cb, qos

        )
    
        self.node.get_logger().info("Waiting for images...")

        while (

            self._rgb_image is None

            or self._depth_image is None

            or self._depth_camera_info is None

        ):

            rclpy.spin_once(self.node, timeout_sec=0.1)
    
        self.node.destroy_subscription(rgb_sub)

        self.node.destroy_subscription(depth_sub)

        self.node.destroy_subscription(info_sub)
    
        self.node.get_logger().info("All images received, calling YOLO...")
    
        rgb_image = self._rgb_image

        depth_image = self._depth_image

        depth_camera_info = self._depth_camera_info
    
        # --- ROS 2: construct request via .Request(), then set fields ---

        yolo_request = YoloDetection3D.Request()

        yolo_request.image_raw = rgb_image

        yolo_request.model = "yolo11n-seg.pt"

        yolo_request.depth_image = depth_image

        yolo_request.depth_camera_info = depth_camera_info

        yolo_request.filter = ["person"]

        yolo_request.target_frame = "map"
    
        yolo_future = yolo_client.call_async(yolo_request)

        rclpy.spin_until_future_complete(self.node, yolo_future)

        yolo_response = yolo_future.result()
    
        # Service call failed entirely, no response to work with

        if yolo_response is None:

            self.node.get_logger().warn("YOLO detection service call failed.")

            return "failed"
    
        # Service succeeded but no people detected, fall back to undetected guest assignment

        if len(yolo_response.detected_objects) == 0:

            self.node.get_logger().warn("No persons detected by YOLO.")

            userdata.named_guest_detection = self._handle_no_detections(

                userdata.guest_data

            )

            return "succeeded"
    
        self.node.get_logger().info(

            f"YOLO detected {len(yolo_response.detected_objects)} person(s)."

        )
    
        cropped_rgb_image = self._crop_image(yolo_response.detected_objects, rgb_image)
    
        recognise_request = Recognise3D.Request()

        recognise_request.image_raw = cropped_rgb_image

        recognise_request.depth_image = depth_image

        recognise_request.depth_camera_info = depth_camera_info

        recognise_request.threshold = 0.5

        recognise_request.target_frame = "map"
    
        try:

            recognise_future = recognise_client.call_async(recognise_request)

            rclpy.spin_until_future_complete(self.node, recognise_future)

            response = recognise_future.result()
    
            if response is None:

                self.node.get_logger().warn("Recognise service call returned None.")

                return "failed"
    
            if len(response.detections) == 0:

                self.node.get_logger().info(

                    "No recognitions returned; falling back to _handle_no_detections."

                )

                named_guest_detection = self._handle_no_detections(userdata.guest_data)

            else:

                detection_id = response.detections[0].name

                if userdata.guest_data.get(detection_id, {}).get(

                    "seating_detection", False

                ):

                    self.node.get_logger().info(

                        f"Guest '{detection_id}' already detected; falling back."

                    )

                    named_guest_detection = self._handle_no_detections(

                        userdata.guest_data

                    )

                else:

                    named_guest_detection = response.detections[0]

                    userdata.guest_data[named_guest_detection.name][

                        "seating_detection"

                    ] = True

                    self.node.get_logger().info(

                        f"Recognised guest: {named_guest_detection.name}"

                    )
    
            userdata.named_guest_detection = named_guest_detection
    
        except Exception as e:

            self.node.get_logger().warn(f"Unable to perform recognition: {str(e)}")

            return "failed"
    
        return "succeeded"
    
