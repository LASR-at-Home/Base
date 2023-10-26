#!/usr/bin/env python3

import rospy
from lasr_vision_msgs.srv import YoloDetection, YoloDetectionRequest
from sensor_msgs.msg import Image


def detect_objects(object_names: [str], model="coco"):
    """
    Detects all persons in an image
    """
    rospy.wait_for_service('yolo_object_detection_server/detect_objects')

    if not isinstance(object_names, list):
        raise ValueError("please input a list of strings")

    try:
        detect_objects = rospy.ServiceProxy('yolo_object_detection_server/detect_objects', YoloDetection)
        image_msg = rospy.wait_for_message('/xtion/rgb/image_raw', Image)  # wait for rgb image

        req = YoloDetectionRequest()
        req.image_raw = image_msg
        req.dataset = model
        req.confidence = 0.7
        req.nms = 0.3

        yolo_resp = detect_objects(req)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return []

    objects = []
    for obj in yolo_resp.detected_objects:
        if obj.name in object_names:
            objects.append(obj)
    return objects


if __name__ == '__main__':
    rospy.init_node("objects_detection", anonymous=True)
    objects = detect_objects(["person", "mug", "phone"])
    for o in objects:
        print("object name: ", o.name)
        print("object confidence: ", o.confidence)
        print("object position: ", o.xywh)

