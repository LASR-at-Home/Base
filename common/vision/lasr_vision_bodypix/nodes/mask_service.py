#!/usr/bin/env python3
import rospy
import lasr_vision_bodypix as bodypix
from lasr_vision_msgs.srv import (
    BodyPixMaskDetection,
    BodyPixMaskDetectionRequest,
    BodyPixMaskDetectionResponse,
)

# Initialise rospy
rospy.init_node("bodypix_mask_service")

# Determine variables
PRELOAD = rospy.get_param("~preload", [])  # resnet50 or mobilenet50

for model in PRELOAD:
    pass


def detect(request: BodyPixMaskDetectionRequest) -> BodyPixMaskDetectionResponse:
    """
    Hand off detection request to bodypix library
    """
    return bodypix.detect_masks(request)


rospy.Service("/bodypix/mask_detection", BodyPixMaskDetection, detect)
rospy.loginfo("BodyPix service started")
rospy.spin()
