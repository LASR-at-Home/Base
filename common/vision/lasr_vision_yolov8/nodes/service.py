#!/usr/bin/env python3

# import re
# import rospy
# import rospkg
# import lasr_vision_yolov8 as yolo

# from typing import Dict

# from sensor_msgs.msg import Image
# from visualization_msgs.msg import Marker

# from lasr_vision_msgs.srv import (
#     YoloDetection,
#     YoloDetectionRequest,
#     YoloDetectionResponse,
#     YoloDetection3D,
#     YoloDetection3DRequest,
#     YoloDetection3DResponse,
# )

# # Put ourselves in the model folder
# import os
# import rospkg

# rp = rospkg.RosPack()
# package_path = rp.get_path("lasr_vision_yolov8")
# os.chdir(os.path.abspath(os.path.join(package_path, "models")))

# # Initialise rospy
# rospy.init_node("yolov8_service")

# # Determine variables
# PRELOAD = rospy.get_param("~preload", [])

# for model in PRELOAD:
#     yolo.load_model(model)

# # Prepare publisher
# debug_publishers: Dict[str, rospy.Publisher] = {}
# debug_publisher = rospy.Publisher("/yolov8/debug", Image, queue_size=1)


# def detect(request: YoloDetectionRequest) -> YoloDetectionResponse:
#     """
#     Hand off detection request to yolo library
#     """
#     if request.dataset in debug_publishers:
#         debug_publisher = debug_publishers[request.dataset]
#     else:
#         topic_name = re.sub(r"[\W_]+", "", request.dataset)
#         debug_publisher = rospy.Publisher(
#             f"/yolov8/debug/{topic_name}", Image, queue_size=1
#         )
#     return yolo.detect(request, debug_publisher)


# def detect_3d(request: YoloDetection3DRequest) -> YoloDetection3DResponse:
#     """
#     Hand off detection request to yolo library
#     """
#     if request.dataset in debug_publishers:
#         debug_inference_publisher, debug_point_publisher = debug_publishers[
#             request.dataset
#         ]
#     else:
#         topic_name = re.sub(r"[\W_]+", "", request.dataset)
#         debug_inference_publisher = rospy.Publisher(
#             f"/yolov8/debug/{topic_name}", Image, queue_size=1
#         )
#         debug_point_publisher = rospy.Publisher(
#             f"/yolov8/debug/points", Marker, queue_size=100
#         )

#     return yolo.detect_3d(request, debug_inference_publisher, debug_point_publisher)


# yolo.start_tf_buffer()
# rospy.Service("/yolov8/detect", YoloDetection, detect)
# rospy.Service("/yolov8/detect3d", YoloDetection3D, detect_3d)
# rospy.loginfo("YOLOv8 service started")
# rospy.spin()

from typing import Dict

import os

import rospy
import rospkg
import image_geometry

import ultralytics
import torch
import numpy as np
import cv2

from lasr_vision_msgs.srv import (
    YoloDetection,
    YoloDetectionRequest,
    YoloDetectionResponse,
    YoloDetection3D,
    YoloDetection3DRequest,
    YoloDetection3DResponse,
)
from lasr_vision_msgs.msg import Detection, Detection3D

import cv2_img
from cv_bridge import CvBridge


class YOLOService:

    _cache: Dict[str, ultralytics.YOLO]

    def __init__(self):

        self._cache = {}
        self._device = rospy.get_param(
            "~device", "cuda:0" if torch.cuda.is_available() else "cpu"
        )
        self._bridge = CvBridge()

        rospy.Service("/yolov8/detect", YoloDetection, self._detect)
        rospy.Service("/yolov8/detect3d", YoloDetection, self._detect3d)

    def _detect(self, req: YoloDetectionRequest) -> YoloDetectionResponse:
        response = YoloDetectionResponse()

        cv_im = self._bridge.imgmsg_to_cv2(req.image_raw, desired_encoding="bgr8")
        cv2.imshow("bgr", cv_im)
        results = self._yolo(cv_im, req.model, req.confidence)

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
        cv2.imshow(results.plot())
        cv2.waitKey(0)
        return response

    def _detect3d(self, req: YoloDetection3DRequest) -> YoloDetection3DResponse:
        response = YoloDetection3DResponse()

        cv_im = self._bridge.imgmsg_to_cv2(req.image_raw, desired_encoding="bgr8")
        results = self._yolo(cv_im, req.model, req.confidence)
        depth_im = self._bridge.imgmsg_to_cv2(
            req.depth_image, desired_encoding="passthrough"
        )

        # Convert input images
        # cv_im = self._bridge.imgmsg_to_cv2(req.image_raw, desired_encoding="bgr8")
        # cv2.imshow("bgr", cv_im)
        # cv2.waitKey(0)
        # cv_image = self._bridge.imgmsg_to_cv2(req.depth_image, "32FC1")
        # cv_image_array = np.array(cv_image, dtype=np.dtype("f8"))
        # cv_image_norm = cv2.normalize(
        #     cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX
        # )
        # # cv_image_resized = cv2.resize(
        # #     cv_image_norm, self.desired_shape, interpolation=cv2.INTER_CUBIC
        # # )
        # cv2.imshow("Depth View", cv_image_norm)
        # cv2.waitKey(0)
        # cv2.imshow("rgb", depth_im)
        # cv2.waitKey(0)
        # # Camera intrinsics
        # camera = image_geometry.PinholeCameraModel()
        # camera.fromCameraInfo(req.depth_camera_info)
        K = req.depth_camera_info.K
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]

        # # Run YOLO
        results = self._yolo(cv_im, req.model, req.confidence)
        # cv2.imshow("yolo", results.plot())
        # cv2.waitKey(0)
        for result in results:
            detection = Detection3D()
            detection.name = result.names[result.boxes.cls.int().item()]
            detection.confidence = result.boxes.conf.item()
            x, y, w, h = (
                result.boxes.xywh.round().int().squeeze().cpu().numpy().tolist()
            )
            detection.xywh = [x, y, w, h]

            center_u = y  # + (h / 2)
            center_v = x  # + (w / 2)

            #     # Compute ROI from mask (if present) or fallback to bounding box
            has_mask = result.masks is not None
            if has_mask:
                detection.xyseg = (
                    np.array(result.masks.xy).flatten().round().astype(int).tolist()
                )
                contours = np.array(detection.xyseg).reshape(-1, 2)
                mask = np.zeros(depth_im.shape[:2], dtype=np.uint8)
                cv2.fillPoly(mask, [contours], color=255)
                roi = cv2.bitwise_and(depth_im, depth_im, mask=mask)
            else:
                u_min = max(center_u - h // 2, 0)
                u_max = min(center_u + h // 2, depth_im.shape[0] - 1)
                v_min = max(center_v - w // 2, 0)
                v_max = min(center_v + w // 2, depth_im.shape[1] - 1)
                roi = depth_im[v_min:v_max, u_min:u_max]

            # roi = roi.astype(np.float32) / 1000.0  # mm → meters

            if not np.any(roi):
                continue

            if has_mask:
                valid_depth = roi[roi > 0]
                if valid_depth.size == 0:
                    continue
                bb_center_z = np.median(valid_depth)
            else:
                bb_center_z = depth_im[center_v, center_u]  # / 1000.0

            z_diff = np.abs(roi - bb_center_z)
            mask_z = z_diff <= 0.3
            mask_z = z_diff >= 0.0
            if not np.any(mask_z):
                continue

            roi_filtered = roi[mask_z]
            z_min, z_max = np.min(roi_filtered), np.max(roi_filtered)
            z = (z_max + z_min) / 2

            if z == 0:
                continue

            # Project center pixel to 3D
            x3d = z * (center_u - cx) / fx
            y3d = z * (center_v - cy) / fy

            detection.point.x = x3d
            detection.point.y = y3d
            detection.point.z = z

            response.detected_objects.append(detection)

        return response

    def _maybe_load_model(self, model_name: str) -> ultralytics.YOLO:
        if model_name in self._cache:
            return self._cache[model_name]

        model = self._cache[model_name] = ultralytics.YOLO(model_name).to(self._device)

        rospy.loginfo(f"Loaded {model_name} model on {self._device}")
        return model

    def _yolo(self, img, model: str, conf: float) -> ultralytics.engine.results.Results:
        yolo = self._maybe_load_model(model)
        results = yolo(img, conf=conf)[0]
        return results


yolov8 = None
marker_pub = None

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


# Define your YoloDetection3DRequest
def detect_3d_callback(img_msg, depth_msg, depth_camera_info_msg):
    # Create the YoloDetection3DRequest message
    req = YoloDetection3DRequest(
        image_raw=img_msg,
        depth_image=depth_msg,
        depth_camera_info=depth_camera_info_msg,
        model="yolo11n-seg.pt",
        confidence=0.9,
    )

    # Create an instance of your yolov8 class and call _detect3d method
    response = yolov8._detect3d(req)

    # Print the response or handle it as needed
    print(response)

    for detection in response.detected_objects:
        # Create a Marker message
        marker = Marker()
        marker.header.frame_id = depth_msg.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.id = 0  # Use detection name or another unique ID
        marker.type = Marker.SPHERE  # We use a sphere to visualize the 3D center
        marker.action = Marker.ADD

        # Set the marker position to the 3D coordinates of the detection center
        marker.pose.position = detection.point

        # Set the marker scale (size of the sphere)
        marker.scale.x = 0.1  # Set the diameter of the sphere
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Set the marker color (e.g., red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully opaque

        # Publish the marker
        marker_pub.publish(marker)


def main():
    rospy.init_node("yolo_3d_node", anonymous=True)
    global yolov8
    yolov8 = (
        YOLOService()
    )  # Ensure you initialize this correctly, possibly with other params
    global marker_pub
    marker_pub = rospy.Publisher("/detection_markers", Marker, queue_size=10)

    # Define the topic names (replace with your actual topic names)
    image_topic = "/xtion/rgb/image_raw"
    depth_topic = "/xtion/depth_registered/image_raw"
    camera_info_topic = "/xtion/depth_registered/camera_info"

    # Create Subscribers using message filters
    image_sub = message_filters.Subscriber(image_topic, Image)
    depth_sub = message_filters.Subscriber(depth_topic, Image)
    camera_info_sub = message_filters.Subscriber(camera_info_topic, CameraInfo)

    # Create a TimeSynchronizer or ApproximateTimeSynchronizer to synchronize the messages
    ts = message_filters.TimeSynchronizer(
        [image_sub, depth_sub, camera_info_sub], 10
    )  # 10 is the queue size
    ts.registerCallback(detect_3d_callback)

    rospy.spin()


if __name__ == "__main__":
    main()
