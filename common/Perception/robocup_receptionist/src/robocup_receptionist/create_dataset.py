#!/usr/bin/env python3

import os
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge3 import CvBridge, cv2

PHOTO_AMOUNT = 20


class CreateDataset:
    def __init__(self):
        self.path = '/home/elisabeth/Desktop/images/host/'
        self.bridge = CvBridge()
    
    def img_cb(self, msg):
        self.cv_img = self.bridge.imgmsg_to_cv2_np(msg)
    
    def main(self):
        rospy.sleep(0.2)
        count = 0
        while count < PHOTO_AMOUNT:
            img_msg = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
            cv_img = self.bridge.imgmsg_to_cv2_np(img_msg)
            cv2.imwrite(f'{self.path}{count}.jpg', cv_img)
            count += 1
            rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('create_dataset', sys.argv)
    m = CreateDataset()
    m.main()
    