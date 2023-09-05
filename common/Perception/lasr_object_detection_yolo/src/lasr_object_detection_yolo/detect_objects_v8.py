#!/usr/bin/env python3

import rospy
from lasr_object_detection_yolo.srv import YoloDetection, YoloDetectionRequest
from sensor_msgs.msg import Image


def detect_objects(object_names: [str], confidence=0.25, nms=0.4, model="yolov8n.pt"):
    """
    Detects all persons in an image using yolov8
    """
    rospy.wait_for_service("/yolov8/detect", rospy.Duration(15.0))

    if not isinstance(object_names, list):
        raise ValueError("please input a list of strings")

    try:
        detect_service = rospy.ServiceProxy('/yolov8/detect', YoloDetection)

        if rospy.get_published_topics(namespace='/camera/image_raw'):
            image_msg = rospy.wait_for_message('/camera/image_raw', Image)  # wait for depth image
        elif rospy.get_published_topics(namespace='/xtion/rgb/image_raw'):
            image_msg = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        elif rospy.get_published_topics(namespace='/usb_cam/image_raw'):
            image_msg = rospy.wait_for_message('/usb_cam/image_raw', Image)  # wait for rgb image


        req = YoloDetectionRequest()
        req.image_raw = image_msg
        req.dataset = model
        req.confidence = confidence
        req.nms = nms
        resp = detect_service(req)
        print(resp)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return []

    objects = []
    for obj in resp.detected_objects:
        if obj.name in object_names:
            objects.append(obj)
    return objects


if __name__ == '__main__':
    rospy.init_node("objects_detection_yolov8", anonymous=True)
    objects = detect_objects(["person", "mug", "phone"])
    for o in objects:
        print("object name: ", o.name)
        print("object confidence: ", o.confidence)
        print("object position: ", o.xywh)
