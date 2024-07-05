#!/usr/bin/env python3
import rospy
import lasr_vision_bodypix as bodypix
from lasr_vision_msgs.srv import (
    BodyPixKeypointDetection,
    BodyPixKeypointDetectionRequest,
    BodyPixKeypointDetectionResponse,
)

# Initialise rospy
rospy.init_node("bodypix_keypoint_service")

# Determine variables
PRELOAD = rospy.get_param("~preload", [])  # resnet50 or mobilenet50

for model in PRELOAD:
    pass


def detect(
    request: BodyPixKeypointDetectionRequest,
) -> BodyPixKeypointDetectionResponse:
    """
    Hand off detection request to bodypix library
    """
    return bodypix.detect_keypoints(request)


rospy.Service("/bodypix/keypoint_detection", BodyPixKeypointDetection, detect)
rospy.loginfo("BodyPix keypoint service started")
rospy.spin()
