from typing import List, Dict, Optional

 

import rclpy
import yasmin
import yasmin_ros
import numpy as np
import cv2

from yasmin import Blackboard
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

 

from lasr_vision_interfaces.msg import Detection3D
from lasr_vision_interfaces.srv import Recognise3D, YoloDetection3D
 

class Recognise(RosState):

 

    _rgb_image_topic: str
    _depth_image_topic: str
    _depth_info_topic: str
    _bridge: CvBridge
    _can_detect_second_guest: bool = False

    def __init__(self, can_detect_second_guest: bool = False):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("guest_data")
        self.add_input_key("guest_seat_point")
        self.add_input_key("seated_guest_locs")
        self.add_output_key("named_guest_detection")
        self.add_output_key("guest_data")

        self.node = yasmin_ros.logger_node
        self._rgb_image_topic = "/head_front_camera/rgb/image_raw"
        self._depth_image_topic = "/head_front_camera/depth/image_raw"
        self._depth_info_topic = "/head_front_camera/depth/camera_info"
        self._can_detect_second_guest = can_detect_second_guest
        self._bridge = CvBridge()

    def _handle_no_detections(self, guest_data: Dict) -> Optional[Detection3D]:
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
        cropped = rgb_image_raw[y : y + h, x : x + w]
        return self._bridge.cv2_to_imgmsg(cropped, encoding="rgb8")

    def execute(self, blackboard: Blackboard) -> str:

        recognise_client = self.node.create_client(

            Recognise3D, "/lasr_vision_reid/recognise/threed"

        )

        yolo_client = self.node.create_client(YoloDetection3D, "/yolo/detect3d")

        if not recognise_client.wait_for_service(timeout_sec=5.0):
            yasmin.YASMIN_LOG_ERROR(
                "Recognise3D service /lasr_vision_reid/recognise/threed not available"

            )

            return "failed"

        if not yolo_client.wait_for_service(timeout_sec=5.0):
            yasmin.YASMIN_LOG_ERROR(
                "YoloDetection3D service /yolo/detect3d not available"

            )

            return "failed"

        # Reset images
        self._rgb_image = None
        self._depth_image = None
        self._depth_camera_info = None

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

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

        yasmin.YASMIN_LOG_INFO("Waiting for images...")
        while (
            self._rgb_image is None
            or self._depth_image is None
            or self._depth_camera_info is None
        ):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_subscription(rgb_sub)
        self.node.destroy_subscription(depth_sub)
        self.node.destroy_subscription(info_sub)

        yasmin.YASMIN_LOG_INFO("All images received, calling YOLO...")

        rgb_image = self._rgb_image
        depth_image = self._depth_image
        depth_camera_info = self._depth_camera_info

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
            yasmin.YASMIN_LOG_WARN("YOLO detection service call failed.")
            return "failed"

        # Service succeeded but no people detected, fall back to undetected guest assignment

        if len(yolo_response.detected_objects) == 0:
            yasmin.YASMIN_LOG_WARN("No persons detected by YOLO.")
            blackboard["named_guest_detection"] = self._handle_no_detections(
                blackboard["guest_data"]
            )

            return "succeeded"

        yasmin.YASMIN_LOG_INFO(
            f"YOLO detected {len(yolo_response.detected_objects)} person(s), cropping image for recognition."
        )
        for det in yolo_response.detected_objects:
            yasmin.YASMIN_LOG_INFO(
                f" - Detected object: {det.name} at {det.xywh} with length {len(det.xyseg)}"
            )

        cropped_rgb_image = self._crop_image(yolo_response.detected_objects, rgb_image)

        debug_img = self._bridge.imgmsg_to_cv2(
            cropped_rgb_image, desired_encoding="rgb8"
        )
        cv2.imwrite("/tmp/cropped_rgb_image.png", debug_img)
        yasmin.YASMIN_LOG_INFO("Cropped RGB image saved for debugging.")

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
                yasmin.YASMIN_LOG_WARN("Recognise service call returned None.")
                return "failed"

            if len(response.detections) == 0:
                yasmin.YASMIN_LOG_INFO(
                    "No recognitions returned; falling back to _handle_no_detections."

                )
                named_guest_detection = self._handle_no_detections(
                    blackboard["guest_data"]
                )
            else:

                detection_id = response.detections[0].name
                if blackboard["guest_data"].get(detection_id, {}).get(
                    "seating_detection", False

                ):
                    yasmin.YASMIN_LOG_INFO(
                        f"Guest '{detection_id}' already detected; falling back."

                    )

                    named_guest_detection = self._handle_no_detections(
                        blackboard["guest_data"]
                    )

                else:

                    named_guest_detection = response.detections[0]
                    blackboard["guest_data"][named_guest_detection.name][
                        "seating_detection"

                    ] = True
                    yasmin.YASMIN_LOG_INFO(
                        f"Recognised guest: {named_guest_detection.name}"

                    )

            blackboard["named_guest_detection"] = named_guest_detection

        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"Unable to perform recognition: {str(e)}")
            return "failed"

        return "succeeded"
    
