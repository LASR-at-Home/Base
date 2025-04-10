from typing import List, Tuple
import cv2
import numpy as np
from shapely.geometry.point import Point as ShapelyPoint
from shapely.geometry.polygon import Polygon as ShapelyPolygon
from shapely.validation import explain_validity
import rospy
from rclpy.wait_for_message import wait_for_message
from cv2_img import cv2_img_to_msg, msg_to_cv2_img
from cv2_pcl import pcl_to_cv2
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point, Polygon, PoseWithCovarianceStamped

from lasr_vision_interfaces.msg import CDRequest, CDResponse, Detection, Detection3D
from lasr_vision_interfaces.srv import (
    CroppedDetection,
    YoloDetection,
    YoloDetection3D,
)

CroppedDetection_Request = CroppedDetection.Request()
CroppedDetection_Response = CroppedDetection.Response()

yolo_client_cache = {}


def _2d_bbox_crop(
    image: np.ndarray,
    crop_method: str,
    detections: List[Detection],
) -> Tuple[List[np.ndarray], List[Detection], List[float]]:
    """Crop 2D detections by the YOLO detection bounding box.

    Args:
        image (np.ndarray): RGB image to crop.
        crop_method (str): Method to crop by. Can be "centered", "right-most", "left-most", "top-most", or "bottom-most".
        detections (List[Detection]): List of 2D detections to crop.

    Returns:
        List[np.ndarray]: List of cropped images.
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

    distances = [
        np.sqrt((x_to_compare - det.xywh[0]) ** 2 + (y_to_compare - det.xywh[1]) ** 2)
        for det in detections
    ]

    detections = [det for _, det in sorted(zip(distances, detections))]
    distances.sort()

    cropped_images = []
    for detection in detections:
        x, y, w, h = (
            detection.xywh[0],
            detection.xywh[1],
            detection.xywh[2],
            detection.xywh[3],
        )
        cropped_images.append(image[y - h // 2 : y + h // 2, x - w // 2 : x + w // 2])

    return cropped_images, detections, distances


def _2d_mask_crop(
    image: np.ndarray,
    crop_method: str,
    detections: List[Detection],
) -> Tuple[List[np.ndarray], np.ndarray, List[Detection], List[float]]:
    """Crop 2D detections by the YOLO detection mask.

    Args:
        image (np.ndarray): RGB image to crop.
        crop_method (str): Method to crop by. Can be "centered", "right-most", "left-most", "top-most", or "bottom-most".
        detections (List[Detection]): List of 2D detections to crop.

    Returns:
        List[np.ndarray]: List of cropped images.
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

    distances = [
        np.sqrt((x_to_compare - det.xywh[0]) ** 2 + (y_to_compare - det.xywh[1]) ** 2)
        for det in detections
    ]

    detections = [det for _, det in sorted(zip(distances, detections))]
    distances.sort()

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

    return masked_images, unified_mask, detections, distances


def _3d_bbox_crop(
    rgb_image: np.ndarray,
    crop_method: str,
    robot_location: Point,
    detections: List[Detection3D],
) -> Tuple[List[np.ndarray], List[Detection3D], List[float]]:
    """Function to crop 3D detections by the YOLO detection bounding box.

    Args:
        point_cloud (np.ndarray): RGBD pointcloud to crop.
        crop_method (str): Method to crop by. Can be "closest" or "furthest".
        robot_location (Point): The robot's current location, used for determining
        the closest/furthest detection.
        detections (List[Detection3D]): List of 3D detections to crop.

    Returns:
        List[np.ndarray]: List of cropped images.
    """

    distances = [
        np.sqrt(
            (robot_location.x - det.point.x) ** 2
            + (robot_location.y - det.point.y) ** 2
            + (robot_location.z - det.point.z) ** 2
        )
        for det in detections
    ]

    if crop_method == "closest":
        detections = [det for _, det in sorted(zip(distances, detections))]
        distances.sort()
    elif crop_method == "furthest":
        detections = [
            det for _, det in sorted(zip(distances, detections), reverse=True)
        ]
        distances.sort(reverse=True)
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

    return cropped_images, detections, distances


def _3d_mask_crop(
    rgb_image: np.ndarray,
    crop_method: str,
    robot_location: Point,
    detections: List[Detection3D],
) -> Tuple[List[np.ndarray], np.ndarray, List[Detection3D], List[float]]:
    """Function to crop 3D detections by the YOLO detection mask.

    Args:
        point_cloud (np.ndarray): RGBD pointcloud to crop.
        crop_method (str): Method to crop by. Can be "closest" or "furthest".
        robot_location (Point): The robot's current location, used for determining
        which detection is closest/furthest.
        detections (List[Detection3D]): List of 3D detections to crop.

    Returns:
        Tuple[List[np.ndarray], np.ndarray, List[Detection3D]]: Tuple of cropped images, the combined mask, and the detections.
    """

    distances = [
        np.sqrt(
            (robot_location.x - det.point.x) ** 2
            + (robot_location.y - det.point.y) ** 2
            + (robot_location.z - det.point.z) ** 2
        )
        for det in detections
    ]

    if crop_method == "closest":
        detections = [det for _, det in sorted(zip(distances, detections))]
        distances.sort()
    elif crop_method == "furthest":
        detections = [
            det for _, det in sorted(zip(distances, detections), reverse=True)
        ]
        distances.sort(reverse=True)
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

    return masked_images, unified_mask, detections, distances


def filter_detections_by_polygon(
    detections: List[Detection3D], polygons: List[Polygon]
) -> Tuple[List[Detection3D], List[int]]:
    """Filters detections by whether their centroid is within a polygon.
    If a detection is within multiple polygons, it will be included multiple times.

    Args:
        detections (List[Detection3D]): List of 3D detections to filter.
        polygons (List[Polygon]): List of geometry_msgs/Polygons to filter by.

    Returns:
        Tuple[List[Detection3D], List[int]]: Tuple of filtered detections and the polygon indices they are within.
        Remember, if a detection is within multiple polygons, it will be included multiple times.
    """

    detection_polygon_ids: List[int] = []
    filtered_detections: List[Detection3D] = []
    for index, polygon in enumerate(polygons):
        area_polygon = ShapelyPolygon([(point.x, point.y) for point in polygon.points])
        print(f"Area polygon: {area_polygon}")
        print(f"Polygon Area: {area_polygon.area}")
        print(f"Polygon is valid: {area_polygon.is_valid}")
        print(explain_validity(area_polygon))
        for detection in detections:
            print(f"Point: {detection.point}")
            if area_polygon.contains(
                ShapelyPoint(detection.point.x, detection.point.y)
            ):
                print(f"Detection {detection} is within polygon {index}")
                detection_polygon_ids.append(index)
                filtered_detections.append(detection)
            else:
                print(f"Detection {detection} is not within polygon {index}")

    return filtered_detections, detection_polygon_ids


def process_single_detection_request(
    node,
    request: CDRequest,
    rgb_image_topic: str = "/xtion/rgb/image_raw",
    depth_image_topic: str = "/xtion/depth_registered/points",
    yolo_2d_service_name: str = "/yolov8/detect",
    yolo_3d_service_name: str = "/yolov8/detect3d",
    robot_pose_topic: str = "/robot_pose",
    debug_topic: str = "/lasr_vision/cropped_detection/debug",
) -> CDResponse:
    """Dispatches a detection request to the appropriate bounding box/mask 2D or 3D cropped
    detection function.

    Args:
        request (CDRequest): The request to process.
        rgb_image_topic (str, optional): The topic to get an RGB image from. Defaults to "/xtion/rgb/image_raw".
        depth_image_topic (str, optional): The topic to getn an RGBD image from. Defaults to "/xtion/depth_registered/points".
        yolo_2d_service_name (str, optional): Name of the 2D Yolo detection service. Defaults to "/yolov8/detect".
        yolo_3d_service_name (str, optional): Name of the 3D Yolo detection service. Defaults to "/yolov8/detect3d".
        robot_pose_topic (str, optional): Service to get the robot's current pose. Defaults to "/robot_pose".
        debug_topic (str, optional): Topic to publish results to for debugging. Defaults to "/lasr_vision/cropped_detection/debug".

    Returns:
        CDResponse: The response to the request.
    """
    valid_2d_crop_methods = [
        "centered",
        "left-most",
        "right-most",
        "top-most",
        "bottom-most",
    ]
    valid_3d_crop_methods = ["closest", "furthest"]
    response = CDResponse()
    combined_mask = None
    if request.method in valid_2d_crop_methods:
        key = "2d"
        if key not in yolo_client_cache:
            yolo_client_cache[key] = node.create_client(
                YoloDetection, yolo_2d_service_name
            )
            while not yolo_client_cache[key].wait_for_service(timeout_sec=1.0):
                node.get_logger().info(
                    f"Service [{yolo_2d_service_name}] not available, waiting..."
                )

        yolo_2d_client = yolo_client_cache[key]

        if request.rgb_image.data:
            rgb_image = request.rgb_image
        else:
            success, rgb_image = wait_for_message(Image, node, rgb_image_topic)
            if not success:
                node.get_logger().error(
                    f"Failed to receive rgb image from {rgb_image_topic}"
                )
                return response

        rgb_cv2 = msg_to_cv2_img(rgb_image)

        req = YoloDetection.Request()
        req.rgb_image = rgb_image
        req.yolo_model = request.yolo_model
        req.yolo_model_confidence = request.yolo_model_confidence
        req.yolo_nms_threshold = request.yolo_nms_threshold

        res = yolo_2d_client.call(req)
        detections = [
            det for det in res.detected_objects if det.name in request.object_names
        ]

        if request.use_mask:
            cropped_images, combined_mask, detections, distances = _2d_mask_crop(
                rgb_cv2, request.method, detections
            )
        else:
            cropped_images, detections, distances = _2d_bbox_crop(
                rgb_cv2, request.method, detections
            )

        response.detections_2d = detections
        if request.return_sensor_reading:
            response.rgb_image = rgb_image

    elif request.method in valid_3d_crop_methods:
        key = "3d"
        if key not in yolo_client_cache:
            yolo_client_cache[key] = node.create_client(
                YoloDetection3D, yolo_3d_service_name
            )
            while not yolo_client_cache[key].wait_for_service(timeout_sec=1.0):
                node.get_logger().info(
                    f"Service [{yolo_3d_service_name}] not available, waiting..."
                )

        yolo_3d_client = yolo_client_cache[key]

        success, robot_pose = wait_for_message(
            PoseWithCovarianceStamped, node, robot_pose_topic
        )
        if not success:
            node.get_logger().error(
                f"Failed to receive robot pose from {robot_pose_topic}"
            )
            return response
        robot_location = robot_pose.pose.pose.position

        if request.pointcloud.data:
            pointcloud_msg = request.pointcloud
        else:
            success, pointcloud_msg = wait_for_message(
                PointCloud2, node, depth_image_topic
            )
            if not success:
                node.get_logger().error(
                    f"Failed to receive pointcloud from {depth_image_topic}"
                )
                return response

        pointcloud_rgb = pcl_to_cv2(pointcloud_msg)

        req = YoloDetection3D.Request()
        req.pointcloud = pointcloud_msg
        req.yolo_model = request.yolo_model
        req.yolo_model_confidence = request.yolo_model_confidence
        req.yolo_nms_threshold = request.yolo_nms_threshold

        res = yolo_3d_client.call(req)
        detections = [
            det for det in res.detected_objects if det.name in request.object_names
        ]

        if request.polygons:
            detections, polygon_ids = filter_detections_by_polygon(
                detections, request.polygons
            )
            response.polygon_ids = polygon_ids

        if request.use_mask:
            cropped_images, combined_mask, detections, distances = _3d_mask_crop(
                pointcloud_rgb, request.method, robot_location, detections
            )
        else:
            cropped_images, detections, distances = _3d_bbox_crop(
                pointcloud_rgb, request.method, robot_location, detections
            )

        response.detections_3d = detections
        if request.return_sensor_reading:
            response.pointcloud = pointcloud_msg

    else:
        node.get_logger().error(f"Invalid crop method: {request.method}")
        return response

    response.cropped_imgs = [
        cv2_img_to_msg(cropped_img) for cropped_img in cropped_images
    ]

    debug_publisher = node.create_publisher(Image, debug_topic, 10)
    closest_pub = node.create_publisher(Image, debug_topic + "_closest", 10)
    combined_mask_debug_publisher = node.create_publisher(
        Image, debug_topic + "_mask", 10
    )

    # Tile the images for debugging purposes
    if combined_mask is not None:
        # Add distances to the image
        for i, (dist, detect) in enumerate(zip(distances, detections)):
            continue
            # cv2.putText(
            #     combined_mask,
            #     f"Dist: {round(dist, 2)}",
            #     (detect.xywh[0], detect.xywh[1]),
            #     cv2.FONT_HERSHEY_SIMPLEX,
            #     1,
            #     (0, 255, 0),
            #     2,
            #     cv2.LINE_AA,
            # )
        combined_mask_debug_publisher.publish(cv2_img_to_msg(combined_mask))
        response.masked_img = cv2_img_to_msg(combined_mask)

    try:
        closest_pub.publish(cv2_img_to_msg(cropped_images[0]))
    except IndexError:
        node.get_logger().warn("No detections found")

    response.distances = distances

    try:
        print("...")
        # debug_image = np.hstack(cropped_images)
        # # Add distances to the image
        # for i, dist in enumerate(distances):
        #     cv2.putText(
        #         debug_image,
        #         f"Dist: {round(dist, 2)}",
        #         (i * cropped_images[0].shape[0] + 150, 50),
        #         cv2.FONT_HERSHEY_SIMPLEX,
        #         1,
        #         (0, 255, 0),
        #         2,
        #         cv2.LINE_AA,
        #     )

        # debug_publisher.publish(cv2_img_to_msg(debug_image))
    except ValueError:
        node.get_logger().warn("No detections found")

    return response


def process_detection_requests(
    node,
    request: CroppedDetection_Request,
    rgb_image_topic: str = "/xtion/rgb/image_raw",
    depth_image_topic: str = "/xtion/depth_registered/points",
    yolo_2d_service_name: str = "/yolov8/detect",
    yolo_3d_service_name: str = "/yolov8/detect3d",
    robot_pose_topic: str = "/robot_pose",
    debug_topic: str = "/lasr_vision/cropped_detection/debug",
) -> CroppedDetection_Response:
    """Processes a list of detection requests.

    Args:
        request (CroppedDetectionRequestSrv): The request to process.
        rgb_image_topic (str, optional): The topic to get an RGB image from. Defaults to "/xtion/rgb/image_raw".
        depth_image_topic (str, optional): The topic to getn an RGBD image from. Defaults to "/xtion/depth_registered/points".
        yolo_2d_service_name (str, optional): Name of the 2D Yolo detection service. Defaults to "/yolov8/detect".
        yolo_3d_service_name (str, optional): Name of the 3D Yolo detection service. Defaults to "/yolov8/detect3d".
        robot_pose_topic (str, optional): Service to get the robot's current pose. Defaults to "/amcl_pose".
        debug_topic (str, optional): Topic to publish results to for debugging. Defaults to "/lasr_vision/cropped_detection/debug".

    Returns:
        CroppedDetectionResponseSrv: The response to the request.
    """
    response = CroppedDetection_Response()
    for req in request.requests:
        response.responses.append(
            process_single_detection_request(
                node,
                req,
                rgb_image_topic,
                depth_image_topic,
                yolo_2d_service_name,
                yolo_3d_service_name,
                robot_pose_topic,
                debug_topic,
            )
        )

    return response
