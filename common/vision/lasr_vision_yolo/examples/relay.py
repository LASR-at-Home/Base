#!/usr/bin/env python3

import rospy
import rosgraph
import message_filters

from sensor_msgs.msg import Image, CameraInfo

from lasr_vision_msgs.srv import (
    YoloDetection,
    YoloDetectionRequest,
    YoloDetection3D,
    YoloDetection3DRequest,
    YoloPoseDetection,
    YoloPoseDetectionRequest,
    YoloPoseDetection3D,
    YoloPoseDetection3DRequest,
)


def topic_exists(topic_name: str):
    master = rosgraph.Master("/rostopic")
    published_topics = master.getPublishedTopics("/")
    return any(topic == topic_name for topic, _ in published_topics)


def relay_2d(image_topic: str, model_name: str, model_list: list):
    yolo = rospy.ServiceProxy("/yolo/detect", YoloDetection)
    yolo.wait_for_service()

    def detect_cb(image: Image):
        req = YoloDetectionRequest(
            image_raw=image,
            model=model_name,
            models=model_list,
            confidence=0.5,
            filter=[],
        )
        response = yolo(req)
        rospy.loginfo(response)

    image_sub = rospy.Subscriber(image_topic, Image, detect_cb, queue_size=10)


def relay_3d(
    image_topic: str,
    depth_topic: str,
    depth_camera_info_topic: str,
    model_name: str,
    model_list: list,
):
    yolo = rospy.ServiceProxy("/yolo/detect3d", YoloDetection3D)
    yolo.wait_for_service()

    def detect_cb(image: Image, depth_image: Image, depth_camera_info: CameraInfo):
        req = YoloDetection3DRequest(
            image_raw=image,
            depth_image=depth_image,
            depth_camera_info=depth_camera_info,
            model=model_name,
            models=model_list,
            confidence=0.5,
            filter=[],
            target_frame="map",
        )
        response = yolo(req)
        rospy.loginfo(response)

    image_sub = message_filters.Subscriber(image_topic, Image)
    depth_sub = message_filters.Subscriber(depth_topic, Image)
    depth_camera_info_sub = message_filters.Subscriber(
        depth_camera_info_topic, CameraInfo
    )
    ts = message_filters.ApproximateTimeSynchronizer(
        [image_sub, depth_sub, depth_camera_info_sub], 10, 0.1
    )
    ts.registerCallback(detect_cb)


def relay_keypoints_2d(image_topic: str, model_name: str, model_list: list):
    yolo = rospy.ServiceProxy("/yolo/detect_pose", YoloPoseDetection)
    yolo.wait_for_service()

    def detect_cb(image: Image):
        req = YoloPoseDetectionRequest(
            image_raw=image,
            model=model_name,
            models=model_list,
            confidence=0.5,
            filter=[],
        )
        response = yolo(req)
        rospy.loginfo(response)

    image_sub = rospy.Subscriber(image_topic, Image, detect_cb, queue_size=10)


def relay_keypoints_3d(
    image_topic: str,
    depth_topic: str,
    depth_camera_info_topic: str,
    model_name: str,
    model_list: list,
):
    yolo = rospy.ServiceProxy("/yolo/detect3d_pose", YoloPoseDetection3D)
    yolo.wait_for_service()

    def detect_cb(image: Image, depth_image: Image, depth_camera_info: CameraInfo):
        req = YoloPoseDetection3DRequest(
            image_raw=image,
            depth_image=depth_image,
            depth_camera_info=depth_camera_info,
            model=model_name,
            models=model_list,
            confidence=0.5,
            filter=[],
        )
        response = yolo(req)
        rospy.loginfo(response)

    image_sub = message_filters.Subscriber(image_topic, Image)
    depth_sub = message_filters.Subscriber(depth_topic, Image)
    depth_camera_info_sub = message_filters.Subscriber(
        depth_camera_info_topic, CameraInfo
    )
    ts = message_filters.ApproximateTimeSynchronizer(
        [image_sub, depth_sub, depth_camera_info_sub], 10, 0.1
    )
    ts.registerCallback(detect_cb)


if __name__ == "__main__":
    rospy.init_node("yolo_relay")

    camera = rospy.get_param("~camera", "xtion")
    model_name = rospy.get_param("~model", "yolo11n-seg.pt")
    model_list = rospy.get_param(
        "~models", ["yolo11n-seg.pt", "lasr.pt"]
    )  # Optional list of models

    image_topic = f"/{camera}/rgb/image_raw"
    if not topic_exists(image_topic):
        image_topic = f"/{camera}/image_raw"
    depth_topic = f"/{camera}/depth_registered/image_raw"
    depth_camera_info_topic = f"/{camera}/depth_registered/camera_info"

    rospy.loginfo(f"Using model: {model_name}")
    rospy.loginfo(f"Using models list: {model_list}")

    if model_name.endswith("pose.pt"):
        if topic_exists(depth_topic):
            relay_keypoints_3d(
                image_topic,
                depth_topic,
                depth_camera_info_topic,
                model_name,
                model_list,
            )
        else:
            rospy.loginfo(
                f"Topic {depth_topic} doesn't exist, so inference will be 2D."
            )
            relay_keypoints_2d(image_topic, model_name, model_list)
    else:
        if topic_exists(depth_topic):
            relay_3d(
                image_topic,
                depth_topic,
                depth_camera_info_topic,
                model_name,
                model_list,
            )
        else:
            rospy.loginfo(
                f"Topic {depth_topic} doesn't exist, so inference will be 2D."
            )
            relay_2d(image_topic, model_name, model_list)

    rospy.spin()
