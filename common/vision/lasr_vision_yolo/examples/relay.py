#!/usr/bin/env python3

import rospy
import rosgraph
from sensor_msgs.msg import Image, CameraInfo
import message_filters

from lasr_vision_msgs.srv import (
    YoloDetection,
    YoloDetectionRequest,
    YoloDetection3D,
    YoloDetection3DRequest,
)


def topic_exists(topic_name: str):
    master = rosgraph.Master("/rostopic")
    published_topics = master.getPublishedTopics("/")
    return any(topic == topic_name for topic, _ in published_topics)


def relay_2d(image_topic: str, model_name: str):

    yolo = rospy.ServiceProxy("/yolo/detect", YoloDetection)
    yolo.wait_for_service()

    def detect_cb(image: Image):
        req = YoloDetectionRequest(
            image_raw=image,
            model=model_name,
            confidence=0.5,
        )
        response = yolo(req)
        rospy.loginfo(response)

    image_sub = rospy.Subscriber(image_topic, Image, detect_cb, queue_size=10)


def relay_3d(
    image_topic: str, depth_topic: str, depth_camera_info_topic: str, model_name: str
):

    yolo = rospy.ServiceProxy("/yolo/detect3d", YoloDetection3D)
    yolo.wait_for_service()

    def detect_cb(image: Image, depth_image: Image, depth_camera_info: CameraInfo):
        req = YoloDetection3DRequest(
            image_raw=image,
            depth_image=depth_image,
            depth_camera_info=depth_camera_info,
            model=model_name,
            confidence=0.5,
        )
        response = yolo(req)
        rospy.loginfo(response)

    image_sub = message_filters.Subscriber(image_topic, Image)
    depth_sub = message_filters.Subscriber(depth_topic, Image)
    depth_camera_info_sub = message_filters.Subscriber(
        depth_camera_info_topic, CameraInfo
    )
    ts = message_filters.TimeSynchronizer(
        [image_sub, depth_sub, depth_camera_info_sub], 10
    )
    ts.registerCallback(detect_cb)


if __name__ == "__main__":
    rospy.init_node("yolo_relay")

    camera = rospy.get_param("~camera", "xtion")
    model_name = rospy.get_param("~model", "yolo11n-seg.pt")

    image_topic = f"/{camera}/rgb/image_raw"
    depth_topic = f"/{camera}/depth_registered/image_raw"
    depth_camera_info_topic = f"/{camera}/depth_registered/camera_info"

    if topic_exists(depth_topic):
        relay_3d(image_topic, depth_topic, depth_camera_info_topic, model_name)
    else:
        rospy.loginfo(f"Topic {depth_topic} doesn't exist, so inference will be 2D.")
        relay_2d(image_topic, model_name)

    rospy.spin()
