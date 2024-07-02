#!/usr/bin/env python3
import os
import rospy

from lasr_vision_cropped_detection.cropped_detection import process_detection_requests
from lasr_vision_msgs.srv import (
    CroppedDetection,
    CroppedDetectionRequest,
    CroppedDetectionResponse,
)


def cropped_detection(req: CroppedDetectionRequest) -> CroppedDetectionResponse:
    rospy.loginfo("Received cropped detection request, dispatching...")
    rgb_topic = (
        "/xtion/rgb/image_raw"
        if "tiago" in os.environ["ROS_MASTER_URI"]
        else "/usb_cam/image_raw"
    )
    response: CroppedDetectionResponse = process_detection_requests(
        req, rgb_image_topic=rgb_topic
    )
    rospy.loginfo("Cropped detection request processed")
    return response


rospy.init_node("cropped_detection_service")
rospy.Service("/vision/cropped_detection", CroppedDetection, cropped_detection)
rospy.loginfo("Cropped Detection service started")
rospy.spin()
