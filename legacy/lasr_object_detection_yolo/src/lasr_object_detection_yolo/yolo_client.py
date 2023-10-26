#!/usr/bin/env python3

import rospy

from lasr_vision_msgs.srv import YoloDetection, YoloDetectionRequest
from sensor_msgs.msg import Image
from cv_bridge3 import CvBridge
import cv2

if __name__ == '__main__':
    rospy.init_node("yolo_client", anonymous=True)
    bridge = CvBridge()
    while not rospy.is_shutdown():
        rospy.wait_for_service('yolo_object_detection_server/detect_objects')
        im = rospy.wait_for_message('/usb_cam/image_raw', Image)
        image = bridge.imgmsg_to_cv2(im, desired_encoding='passthrough')
        cv2.imwrite('hello.jpg', image)
        req = YoloDetectionRequest()
        req.image_raw= im
        req.dataset ='coco'
        req.confidence = 0.5
        req.nms = 0.3
        print('hello')
        server = rospy.ServiceProxy('yolo_object_detection_server/detect_objects', YoloDetection)
        detections = server(req).detected_objects
        print(
            f"DETECTED:{detections}\n"
        )