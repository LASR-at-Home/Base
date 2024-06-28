#!/usr/bin/env python3
from typing import List, Tuple
import numpy as np
import cv2
import rospy

from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point, PoseWithCovarianceStamped

from lasr_vision_msgs.msg import Detection, Detection3D
from lasr_vision_msgs.srv import (
    YoloDetection,
    YoloDetection3D,
    CroppedDetectionRequest,
    CroppedDetectionResponse,
)
from cv2_img import cv2_img_to_msg, msg_to_cv2_img
from cv2_pcl import pcl_to_cv2


def _2d_bbox_crop(
    image: np.ndarray,
    crop_method: str,
    detections: List[Detection],
) -> Tuple[List[np.ndarray], List[Detection]]:
    """_summary_

    Args:
        image (np.ndarray): _description_
        crop_method (str): _description_
        detections (List[Detection]): _description_

    Returns:
        List[np.ndarray]: _description_
    """

    if crop_method == "centered":
        y_to_compare = image.shape[0] // 2
        x_to_compare = image.shape[1] // 2
    elif crop_method == "right-most":
        x_to_compare = 0
        y_to_compare = image.shape[0] // 2
    elif crop_method == "left-most":
        x_to_compare = image.shape[1]
        y_to_compare = image.shape[0] // 2
    elif crop_method == "top-most":
        x_to_compare = image.shape[1] // 2
        y_to_compare = 0
    elif crop_method == "bottom-most":
        x_to_compare = image.shape[1] // 2
        y_to_compare = image.shape[0]
    else:
        raise ValueError(f"Invalid 2D crop_method: {crop_method}")

    if len(detections) == 0:
        raise ValueError("No detections found")

    detections.sort(
        key=lambda det: np.sqrt(
            (x_to_compare - det.xywh[0]) ** 2 + (y_to_compare - det.xywh[1]) ** 2
        )
    )

    cropped_images = []
    for detection in detections:
        x, y, w, h = (
            detection.xywh[0],
            detection.xywh[1],
            detection.xywh[2],
            detection.xywh[3],
        )
        cropped_images.append(image[y - h // 2 : y + h // 2, x - w // 2 : x + w // 2])

    return cropped_images, detections


def _2d_mask_crop(
    image: np.ndarray,
    crop_method: str,
    detections: List[Detection],
) -> Tuple[List[np.ndarray], np.ndarray, List[Detection]]:
    """_summary_

    Args:
        image (np.ndarray): _description_
        crop_method (str): _description_
        detections (List[Detection]): _description_

    Returns:
        List[np.ndarray]: _description_
    """

    # Keeping this in a separate function as might want to make function
    # more complex, i.e., add noise to other detections rather than filling
    # in the whole image, etc.

    if crop_method == "centered":
        y_to_compare = image.shape[0] // 2
        x_to_compare = image.shape[1] // 2
    elif crop_method == "right-most":
        x_to_compare = 0
        y_to_compare = image.shape[0] // 2
    elif crop_method == "left-most":
        x_to_compare = image.shape[1]
        y_to_compare = image.shape[0] // 2
    elif crop_method == "top-most":
        x_to_compare = image.shape[1] // 2
        y_to_compare = 0
    elif crop_method == "bottom-most":
        x_to_compare = image.shape[1] // 2
        y_to_compare = image.shape[0]
    else:
        raise ValueError(f"Invalid 2D crop_method: {crop_method}")

    if len(detections) == 0:
        raise ValueError("No detections found")

    if len(detections[0].xyseg) == 0:
        raise ValueError("No segmentation found")

    detections.sort(
        key=lambda det: np.sqrt(
            (x_to_compare - det.xywh[0]) ** 2 + (y_to_compare - det.xywh[1]) ** 2
        )
    )

    masked_images = []
    unified_mask = np.zeros(image.shape).astype(image.dtype)
    for detection in detections:
        # x,y coords of the detection
        # Taken from https://stackoverflow.com/questions/37912928/fill-the-outside-of-contours-opencv
        mask = np.array(detection.xyseg).reshape(-1, 2)
        stencil = np.zeros(image.shape).astype(image.dtype)
        colour = (255, 255, 255)
        cv2.fillPoly(stencil, [mask], colour)
        # Bitwise AND with 0s is 0s, hence we get the image only where the mask is
        # with black elsewhere.
        masked_image = cv2.bitwise_and(image, stencil)
        unified_mask = cv2.bitwise_or(unified_mask, masked_image)
        masked_images.append(masked_image)
    return masked_images, unified_mask, detections


def _3d_bbox_crop(
    rgb_image: np.ndarray,
    crop_method: str,
    robot_location: Point,
    detections: List[Detection3D],
) -> Tuple[List[np.ndarray], List[Detection3D]]:
    """_summary_

    Args:
        point_cloud (np.ndarray): _description_
        crop_method (str): _description_
        robot_location (Point): _description_
        detections (List[Detection3D]): _description_

    Returns:
        List[np.ndarray]: _description_
    """

    if len(detections) == 0:
        raise ValueError("No detections found")

    if crop_method == "closest":
        detections.sort(
            key=lambda det: np.sqrt(
                (robot_location.x - det.point.x) ** 2
                + (robot_location.y - det.point.y) ** 2
                + (robot_location.z - det.point.z) ** 2
            )
        )
    elif crop_method == "furthest":
        detections.sort(
            key=lambda det: np.sqrt(
                (robot_location.x - det.point.x) ** 2
                + (robot_location.y - det.point.y) ** 2
                + (robot_location.z - det.point.z) ** 2
            ),
            reverse=True,
        )
    else:
        raise ValueError(f"Invalid 3D crop_method: {crop_method}")

    cropped_images = []
    for detection in detections:
        x, y, w, h = (
            detection.xywh[0],
            detection.xywh[1],
            detection.xywh[2],
            detection.xywh[3],
        )
        cropped_images.append(
            rgb_image[y - h // 2 : y + h // 2, x - w // 2 : x + w // 2]
        )

    return cropped_images, detections


def _3d_mask_crop(
    rgb_image: np.ndarray,
    crop_method: str,
    robot_location: Point,
    detections: List[Detection3D],
) -> Tuple[List[np.ndarray], np.ndarray, List[Detection3D]]:
    """_summary_

    Args:
        point_cloud (np.ndarray): _description_
        crop_method (str): _description_
        robot_location (Point): _description_
        detections (List[Detection3D]): _description_

    Returns:
        Tuple[List[np.ndarray], np.ndarray, List[Detection3D]]: _description_
    """

    if len(detections) == 0:
        raise ValueError("No detections found")

    if crop_method == "closest":
        detections.sort(
            key=lambda det: np.sqrt(
                (robot_location.x - det.point.x) ** 2
                + (robot_location.y - det.point.y) ** 2
                + (robot_location.z - det.point.z) ** 2
            )
        )
    elif crop_method == "furthest":
        detections.sort(
            key=lambda det: np.sqrt(
                (robot_location.x - det.point.x) ** 2
                + (robot_location.y - det.point.y) ** 2
                + (robot_location.z - det.point.z) ** 2
            ),
            reverse=True,
        )
    else:
        raise ValueError(f"Invalid 3D crop_method: {crop_method}")

    masked_images = []
    unified_mask = np.zeros(rgb_image.shape).astype(rgb_image.dtype)

    for detection in detections:
        # x,y coords of the detection
        # Taken from https://stackoverflow.com/questions/37912928/fill-the-outside-of-contours-opencv
        mask = np.array(detection.xyseg).reshape(-1, 2)
        stencil = np.zeros(rgb_image.shape).astype(rgb_image.dtype)
        colour = (255, 255, 255)
        cv2.fillPoly(stencil, [mask], colour)
        # Bitwise AND with 0s is 0s, hence we get the image only where the mask is
        # with black elsewhere.
        masked_image = cv2.bitwise_and(rgb_image, stencil)
        unified_mask = cv2.bitwise_or(unified_mask, masked_image)
        masked_images.append(masked_image)

    return masked_images, unified_mask, detections


def process_detection_request(
    request: CroppedDetectionRequest,
    rgb_image_topic: str = "/xtion/rgb/image_raw",
    depth_image_topic: str = "/xtion/depth/image_raw",
    yolo_2d_service_name: str = "/yolov8/detect",
    yolo_3d_service_name: str = "/yolov8/detect3d",
    robot_pose_topic: str = "/amcl_pose",
    debug_topic: str = "/lasr_vision/cropped_detection/debug",
) -> CroppedDetectionResponse:
    valid_2d_crop_methods = [
        "centered",
        "left-most",
        "right-most",
        "top-most",
        "bottom-most",
    ]
    valid_3d_crop_methods = ["closest", "furthest"]
    response = CroppedDetectionResponse()
    combined_mask = None
    if request.method in valid_2d_crop_methods:
        yolo_2d_service = rospy.ServiceProxy(yolo_2d_service_name, YoloDetection)
        yolo_2d_service.wait_for_service()
        rgb_image = rospy.wait_for_message(rgb_image_topic, Image)
        rgb_cv2 = msg_to_cv2_img(rgb_image)
        detections = yolo_2d_service(
            rgb_image,
            request.yolo_model,
            request.yolo_model_confidence,
            request.yolo_nms_threshold,
        ).detected_objects
        detections = [det for det in detections if det.name in request.object_names]
        if request.use_mask:
            cropped_images, combined_mask, detections = _2d_mask_crop(
                rgb_cv2, request.method, detections
            )
        else:
            cropped_images, detections = _2d_bbox_crop(
                rgb_cv2, request.method, detections
            )
        response.detections_2d = detections
    elif request.method in valid_3d_crop_methods:
        yolo_3d_service = rospy.ServiceProxy(yolo_3d_service_name, YoloDetection3D)
        yolo_3d_service.wait_for_service()
        robot_pose = rospy.wait_for_message(robot_pose_topic, PoseWithCovarianceStamped)
        robot_location = robot_pose.pose.pose.position
        pointcloud_msg = rospy.wait_for_message(depth_image_topic, PointCloud2)
        pointcloud_rgb = pcl_to_cv2(pointcloud_msg)
        detections = yolo_3d_service(
            pointcloud_msg,
            request.yolo_model,
            request.yolo_model_confidence,
            request.yolo_nms_threshold,
        ).detected_objects
        detections = [det for det in detections if det.name in request.object_names]
        if request.use_mask:
            cropped_images, combined_mask, detections = _3d_mask_crop(
                pointcloud_rgb,
                request.method,
                robot_location,
                detections,
            )
        else:
            cropped_images, detections = _3d_bbox_crop(
                pointcloud_rgb,
                request.method,
                request.robot_location,
                detections,
            )
        response.detections_3d = detections
    else:
        rospy.logerr(f"Invalid crop method: {request.method}")
        return response

    response.cropped_imgs = [
        cv2_img_to_msg(cropped_img) for cropped_img in cropped_images
    ]

    debug_publisher = rospy.Publisher(debug_topic, Image, queue_size=10)
    combined_mask_debug_publisher = rospy.Publisher(
        debug_topic + "_mask", Image, queue_size=10
    )

    # Tile the images for debugging purposes
    if combined_mask is not None:
        combined_mask_debug_publisher.publish(cv2_img_to_msg(combined_mask))

    debug_image = np.hstack(cropped_images)
    debug_publisher.publish(cv2_img_to_msg(debug_image))

    if combined_mask is not None:
        response.masked_img = cv2_img_to_msg(combined_mask)

    return response
