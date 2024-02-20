import rospy
import cv2_img
import numpy as np

from PIL import Image
from ultralytics import YOLO

from sensor_msgs.msg import Image as SensorImage
from lasr_vision_msgs.msg import Detection, Detection3D
from lasr_vision_msgs.srv import (
    YoloDetectionRequest,
    YoloDetectionResponse,
    YoloDetection3DRequest,
    YoloDetection3DResponse,
)
import cv2
import ros_numpy as rnp

from geometry_msgs.msg import Point, PointStamped

import tf2_ros as tf
import tf2_geometry_msgs

from visualization_msgs.msg import Marker


def create_point_marker(x, y, z, idx, g=1.0):
    marker_msg = Marker()
    marker_msg.header.frame_id = "map"
    marker_msg.header.stamp = rospy.Time.now()
    marker_msg.id = idx
    marker_msg.type = Marker.SPHERE
    marker_msg.action = Marker.ADD
    marker_msg.pose.position.x = x
    marker_msg.pose.position.y = y
    marker_msg.pose.position.z = z
    marker_msg.pose.orientation.w = 1.0
    marker_msg.scale.x = 0.1
    marker_msg.scale.y = 0.1
    marker_msg.scale.z = 0.1
    marker_msg.color.a = 1.0
    marker_msg.color.r = 0.0
    marker_msg.color.g = g
    marker_msg.color.b = 0.0
    return marker_msg


# model cache
loaded_models = {}


def load_model(dataset: str) -> None:
    """
    Load a model into cache
    """

    model = None
    if dataset in loaded_models:
        model = loaded_models[dataset]
    else:
        model = YOLO(dataset)
        rospy.loginfo(f"Loaded {dataset} model")

        loaded_models[dataset] = model

    return model


def detect(
    request: YoloDetectionRequest, debug_publisher: rospy.Publisher
) -> YoloDetectionResponse:
    """
    Run YOLO inference on given detection request
    """

    # decode the image
    rospy.loginfo("Decoding")
    img = cv2_img.msg_to_pillow_img(request.image_raw)

    # load model
    rospy.loginfo("Loading model")
    model = load_model(request.dataset)

    # run inference
    rospy.loginfo("Running inference")
    results = model(img, conf=request.confidence, iou=request.nms)
    result = results[0]
    rospy.loginfo("Inference complete")

    # construct response
    detected_objects = []
    object_count = result.boxes.cls.size(dim=0)
    has_segment_masks = result.masks is not None
    for i in range(0, object_count):
        detection = Detection()
        detection.name = result.names[int(result.boxes.cls[i])]
        detection.confidence = float(result.boxes.conf.cpu().numpy()[i])
        detection.xywh = result.boxes.xywh[i].cpu().numpy().astype(int).tolist()

        # copy segmented mask if available
        if has_segment_masks:
            detection.xyseg = result.masks.xy[i].flatten().astype(int).tolist()

        detected_objects.append(detection)

    # publish to debug topic
    if debug_publisher is not None:
        debug_publisher.publish(cv2_img.cv2_img_to_msg(result.plot()))

    response = YoloDetectionResponse()
    response.detected_objects = detected_objects
    return response


def detect_3d(
    request: YoloDetection3DRequest,
    debug_inference_publisher: rospy.Publisher,
    debug_point_publisher: rospy.Publisher,
) -> YoloDetection3DResponse:
    """
    Run YOLO 3D inference on given detection request
    """

    rospy.loginfo("Waiting for transformation to become available")
    tf_buffer = tf.Buffer()
    # Wait for the transformation to become available
    while not tf_buffer.can_transform(
        "map", request.pcl.header.frame_id, rospy.Time(0)
    ):
        rospy.sleep(0.01)

    # Extract rgb image from pointcloud
    rospy.loginfo("Decoding")
    img = np.fromstring(request.pcl.data, dtype=np.uint8)
    img = img.reshape(request.pcl.height, request.pcl.width, 32)
    img = img[:, :, 16:19]

    # Ensure array is contiguous
    img = np.ascontiguousarray(img, dtype=np.uint8)

    # load model
    rospy.loginfo("Loading model")
    model = load_model(request.dataset)

    # run inference
    rospy.loginfo("Running inference")
    results = model(img, conf=request.confidence, iou=request.nms)
    result = results[0]
    rospy.loginfo("Inference complete")

    # construct response
    detected_objects = []
    object_count = result.boxes.cls.size(dim=0)
    has_segment_masks = result.masks is not None
    for i in range(0, object_count):
        detection = Detection3D()
        detection.name = result.names[int(result.boxes.cls[i])]
        detection.confidence = float(result.boxes.conf.cpu().numpy()[i])
        detection.xywh = result.boxes.xywh[i].cpu().numpy().astype(int).tolist()

        # copy segmented mask if available
        if has_segment_masks:
            detection.xyseg = result.masks.xy[i].flatten().astype(int).tolist()

            # Convert xyseg to contours
            contours = np.array(detection.xyseg).reshape(-1, 2)

            # Compute mask from contours
            mask = np.zeros(shape=img.shape[:2])
            cv2.fillPoly(mask, pts=[contours], color=(255, 255, 255))

            # Extract mask indices from bounding box
            indices = np.argwhere(mask)

            if indices.shape[0] == 0:
                return np.full(3, np.nan)

            # Unpack pointcloud into xyz array
            pcl_xyz = rnp.point_cloud2.pointcloud2_to_xyz_array(
                request.pcl, remove_nans=False
            )

            # Extract points of interest
            xyz_points = [pcl_xyz[x][y] for x, y in indices]

            point = Point(*np.nanmean(xyz_points, axis=0))
            point_stamped = PointStamped()
            point_stamped.point = point
            point_stamped.header.frame_id = request.pcl.header.frame_id
            detection.point = tf_buffer.transform(
                point_stamped, request.target_frame
            ).point

            if debug_point_publisher is not None:
                marker = create_point_marker(
                    detection.point.x, detection.point.y, detection.point.z, i
                )
                debug_point_publisher.publish(marker)

        detected_objects.append(detection)

    # publish to debug topic
    if debug_inference_publisher is not None:
        debug_inference_publisher.publish(cv2_img.cv2_img_to_msg(result.plot()))

    response = YoloDetection3DResponse()
    response.detected_objects = detected_objects
    return response
