#!/usr/bin/env python3
import rospy
from typing import List
from lasr_vision_clip.clip_utils import load_model, encode_img
from lasr_vision_msgs.srv import (
    ClipImageEncoder,
    ClipImageEncoderResponse,
    ClipImageEncoderRequest,
)
from sensor_msgs.msg import Image
from cv2_img import msg_to_cv2_img


if __name__ == "__main__":
    rospy.init_node("clip_encoder_test")
    img_topic = "/usb_cam/image_raw"
    rospy.wait_for_service("/clip/img_encoder")
    clip_encoder = rospy.ServiceProxy("/clip/img_encoder", ClipImageEncoder)
    while not rospy.is_shutdown():
        img_msg = rospy.wait_for_message(img_topic, Image)
        request = ClipImageEncoderRequest(image_raw=img_msg)
        response = clip_encoder(request)
        rospy.loginfo(f"Received response: {response}")

    rospy.spin()
