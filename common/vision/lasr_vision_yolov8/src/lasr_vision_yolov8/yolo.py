import rospy
import cv2_img
import cv2_pcl
import numpy as np

from ultralytics import YOLO

from lasr_vision_msgs.msg import Detection, Detection3D
from lasr_vision_msgs.srv import (
    YoloDetectionRequest,
    YoloDetectionResponse,
    YoloDetection3DRequest,
    YoloDetection3DResponse,
)

import markers

from geometry_msgs.msg import Point, PointStamped

import tf2_ros as tf
import tf2_geometry_msgs  # noqa

# global tf buffer
tf_buffer = tf.Buffer(cache_time=rospy.Duration(10))


def start_tf_buffer() -> None:
    tf.TransformListener(tf_buffer)


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

    # Extract rgb image from pointcloud
    rospy.loginfo("Decoding")
    img = cv2_pcl.pcl_to_cv2(request.pcl)

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

        # copy segmented mask if available, and estimate 3D position
        if has_segment_masks:
            detection.xyseg = result.masks.xy[i].flatten().astype(int).tolist()

            centroid = cv2_pcl.seg_to_centroid(request.pcl, np.array(detection.xyseg))

            point_stamped = PointStamped()
            point_stamped.point = Point(*centroid)
            point_stamped.header.frame_id = request.pcl.header.frame_id
            point_stamped.header.stamp = rospy.Time(0)

            # TODO: handle tf errors properly
            while not rospy.is_shutdown():
                try:
                    point_stamped = tf_buffer.transform(point_stamped, "map")
                    detection.point = point_stamped.point
                    break
                except Exception as e:
                    rospy.logerr(e)
                    continue

            # publish to debug topic
            if debug_point_publisher is not None:
                markers.create_and_publish_marker(debug_point_publisher, point_stamped)

        detected_objects.append(detection)

    # publish to debug topic
    if debug_inference_publisher is not None:
        debug_inference_publisher.publish(cv2_img.cv2_img_to_msg(result.plot()))

    response = YoloDetection3DResponse()
    response.detected_objects = detected_objects
    return response
