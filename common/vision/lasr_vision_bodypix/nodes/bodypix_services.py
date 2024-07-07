#!/usr/bin/env python3
import rospy
import lasr_vision_bodypix as bodypix
from lasr_vision_msgs.srv import (
    BodyPixMaskDetection,
    BodyPixMaskDetectionRequest,
    BodyPixMaskDetectionResponse,
    BodyPixKeypointDetection,
    BodyPixKeypointDetectionRequest,
    BodyPixKeypointDetectionResponse,
    DetectWave,
    DetectWaveRequest,
    DetectWaveResponse,
)

from typing import Union
from sensor_msgs.msg import Image
import ros_numpy as rnp
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
import numpy as np
from cv2_pcl import pcl_to_img_msg

# Initialise rospy
rospy.init_node("bodypix_mask_service")

# Determine variables
PRELOAD = rospy.get_param("~preload", [])  # List of models to preload

for model in PRELOAD:
    bodypix.load_model_cached(model)


def detect_masks(request: BodyPixMaskDetectionRequest) -> BodyPixMaskDetectionResponse:
    """
    Hand off detection request to bodypix library
    """
    return bodypix.detect_masks(request)


def detect_keypoints(
    request: BodyPixKeypointDetectionRequest,
) -> BodyPixKeypointDetectionResponse:
    """
    Hand off detection request to bodypix library
    """
    return bodypix.detect_keypoints(request)


def detect_wave(
    request: DetectWaveRequest,
    debug_publisher: Union[rospy.Publisher, None] = rospy.Publisher(
        "debug_waving", Image, queue_size=1
    ),
) -> DetectWaveResponse:
    """
    Detects a waving gesture by checking if the wrist is above the shoulder
    """
    try:
        bp_req = BodyPixKeypointDetectionRequest()
        bp_req.image_raw = pcl_to_img_msg(request.pcl_msg)
        bp_req.dataset = request.dataset
        bp_req.confidence = request.confidence

        detected_keypoints = bodypix.detect_keypoints(bp_req).keypoints
    except Exception as e:
        rospy.logerr(f"Error detecting keypoints: {e}")
        return DetectWaveResponse()

    gesture_to_detect = None

    keypoint_info = {
        keypoint.keypoint_name: {"x": keypoint.x, "y": keypoint.y}
        for keypoint in detected_keypoints
    }
    if "leftShoulder" in keypoint_info and "leftWrist" in keypoint_info:
        if keypoint_info["leftWrist"]["y"] < keypoint_info["leftShoulder"]["y"]:
            gesture_to_detect = "raising_left_arm"

    if "rightShoulder" in keypoint_info and "rightWrist" in keypoint_info:
        if keypoint_info["rightWrist"]["y"] < keypoint_info["rightShoulder"]["y"]:
            gesture_to_detect = "raising_right_arm"

    if gesture_to_detect is not None:
        rospy.loginfo(f"Detected gesture: {gesture_to_detect}")

    wave_point = keypoint_info.get(
        "leftShoulder" if gesture_to_detect == "raising_left_arm" else "rightShoulder"
    )
    # get the pcl instead and transform it to img msg in the beginnign
    pcl_xyz = rnp.point_cloud2.pointcloud2_to_xyz_array(
        request.pcl_msg, remove_nans=False
    )
    try:
        wave_position = np.zeros(3)
        # take the average of the points around the detected keypoint
        for i in range(-5, 5):
            for j in range(-5, 5):
                if np.any(
                    np.isnan(
                        pcl_xyz[int(wave_point["y"]) + i][int(wave_point["x"]) + j]
                    )
                ):
                    rospy.logwarn("Nan point in pcl")
                    continue
                wave_position += pcl_xyz[int(wave_point["y"]) + i][
                    int(wave_point["x"]) + j
                ]
        wave_position /= 100

        wave_position = PointStamped(
            point=Point(*wave_position),
            header=Header(frame_id=request.pcl_msg.header.frame_id),
        )
        rospy.loginfo(f"Wave point: {wave_position}")
    except Exception as e:
        rospy.logerr(f"Error getting wave point: {e}")
        wave_position = PointStamped()

    # if debug_publisher is not None:
    #     cv2_gesture_img = cv2_img.msg_to_cv2_img(request.pcl_msg)
    #     # Add text to the image
    #     cv2.putText(
    #         cv2_gesture_img,
    #         gesture_to_detect,
    #         (10, 30),
    #         cv2.FONT_HERSHEY_SIMPLEX,
    #         1,
    #         (0, 255, 0),
    #         2,
    #         cv2.LINE_AA,
    #     )
    #     # Publish the image
    #     debug_publisher.publish(cv2_img.cv2_img_to_msg(cv2_gesture_img))
    #     create_and_publish_marker(marker_pub, wave_position, r=0, g=1, b=0)

    is_waving = False if gesture_to_detect is None else True

    return DetectWaveResponse(
        keypoints=detected_keypoints,
        wave_detected=is_waving,
        wave_position=wave_position,
    )


rospy.Service("/bodypix/mask_detection", BodyPixMaskDetection, detect_masks)
rospy.Service("/bodypix/keypoint_detection", BodyPixKeypointDetection, detect_keypoints)
rospy.Service("/bodypix/detect_wave", DetectWave, detect_wave)
rospy.loginfo("BodyPix service started")
rospy.spin()
