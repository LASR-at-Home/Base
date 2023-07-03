#!/usr/bin/env python3
import smach
import rospy
from sensor_msgs.msg import PointCloud2, Image
from lasr_object_detection_yolo.srv import YoloDetection

class WaitForPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.detect = rospy.ServiceProxy("yolo_object_detection_server/detect_objects", YoloDetection)
    def execute(self, userdata):
        person_found = False

        while not person_found:
            img_msg = rospy.wait_for_message("/xtion/rgb/image_raw", Image)
            detections = self.detect(img_msg, "coco", 0.7, 0.3)
            person_found == "person" in [det.name for det in detections.detected_objects]
        return 'done'
