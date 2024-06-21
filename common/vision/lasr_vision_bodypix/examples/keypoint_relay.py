#!/usr/bin/env python3

import sys
import rospy
import threading

from sensor_msgs.msg import Image
from lasr_vision_msgs.srv import (
    BodyPixKeypointDetection,
    BodyPixKeypointDetectionRequest,
)

if len(sys.argv) < 2:
    print(
        "Usage: rosrun lasr_vision_bodypix keypoint_relay.py <source_topic> [resnet50|mobilenet50|...]"
    )
    exit()

# figure out what we are listening to
listen_topic = sys.argv[1]

# figure out what model we are using
if len(sys.argv) >= 3:
    model = sys.argv[2]
else:
    model = "resnet50"

processing = False


def detect(image):
    global processing
    processing = True
    rospy.loginfo("Received image message")

    try:
        detect_service = rospy.ServiceProxy(
            "/bodypix/keypoint_detection", BodyPixKeypointDetection
        )
        req = BodyPixKeypointDetectionRequest()
        req.image_raw = image
        req.dataset = model
        req.confidence = 0.7

        resp = detect_service(req)
        print(resp)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
    finally:
        processing = False


def image_callback(image):
    global processing
    if processing:
        return

    t = threading.Thread(target=detect, args=(image,))
    t.start()


def listener():
    rospy.init_node("image_listener", anonymous=True)
    rospy.wait_for_service("/bodypix/keypoint_detection")
    rospy.Subscriber(listen_topic, Image, image_callback)
    rospy.spin()


if __name__ == "__main__":
    listener()
