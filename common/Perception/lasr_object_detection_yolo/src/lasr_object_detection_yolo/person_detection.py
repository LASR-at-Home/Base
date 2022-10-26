#!/usr/bin/env python3

import rospy
from lasr_object_detection_yolo.srv import YoloDetection, YoloDetectionRequest
from sensor_msgs.msg import PointCloud2, Image


def detect_person(model="coco"):
    """
    Detects all persons in an image
    """
    rospy.wait_for_service('yolo_object_detection_server/detect_objects')

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

    people = []
    for obj in yolo_resp.detected_objects:
        if obj.name == 'person':
            people.append(obj)
    return people


if __name__ == '__main__':
    rospy.init_node("person_detection", anonymous=True)
    people = detect_person()
    for p in people:
        print("object name: ", p.name)
        print("object confidence: ", p.confidence)
        print("object position: ", p.xywh)

