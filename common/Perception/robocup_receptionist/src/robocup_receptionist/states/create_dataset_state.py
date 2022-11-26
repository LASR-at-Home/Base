#!/usr/bin/env python3
# coding=utf-8
import os
import shutil
import smach
import rospy
import rospkg
from smach import CBState
from smach_ros import IntrospectionServer
from sensor_msgs.msg import Image
import rospkg
import sys
# from recognize import recognize
from face_detection.train_model import train
from cv_bridge3 import CvBridge
from cv_bridge3 import cv2
from robocup_receptionist.models.guest import Guest
import random
import string
from robocup_receptionist.utils.servers_clients import detect_person
import numpy as np

IMAGE_NUMBER = 20
SLEEP_TIME = 0.8
MAX_DURATION = IMAGE_NUMBER*SLEEP_TIME + 2

'''Creates a dataset of a person and trains a model to redetect that individual. 
   To be able to train, the dataset must contain at least 2 people
'''
class CreateDatasetState(smach.State):
    def __init__(self, base_contoller):
        smach.State.__init__(self,
                             outcomes=['finished_scan', 'scan_failed'],
                             output_keys=['path']
        )
        self.name = ""
        self.path = ""
        self.dir_path = self.path = os.path.join(rospkg.RosPack().get_path('people_detection'), 'dataset')
        self.bridge = CvBridge()
        self.images_taken = 0
        self.base_controller = base_contoller
        self.semaphore = False

        # Choose topic
        if rospy.get_published_topics(namespace='/xtion'):
            self.topic = '/xtion/rgb/image_raw'
        else:
            self.topic = '/usb_cam/image_raw'

        self.sub = rospy.Subscriber(self.topic, Image, self.img_callback)

    def img_callback(self, msg):
        if self.semaphore:
            if rospy.Time.now().to_sec() - self.last_time.to_sec() >= SLEEP_TIME and self.images_taken < IMAGE_NUMBER:
                cv_image = self.bridge.imgmsg_to_cv2_np(msg)
                cv2.imwrite(os.path.join(self.path, f'{self.images_taken}.jpg'), cv_image)
                self.images_taken += 1
                print("*" *30," - IMAGE IS TAKEN ", self.images_taken)
                self.last_time = rospy.Time.now()
            elif self.images_taken >= IMAGE_NUMBER or rospy.Time.now().to_sec() - self.start_time.to_sec() > MAX_DURATION:
                self.semaphore = False

    def execute(self, userdata):
        rand = ''.join(random.choice(string.ascii_lowercase) for _ in range(7))
        userdata.path = self.path = os.path.join(rospkg.RosPack().get_path('people_detection'),  'dataset', rand)

        if os.path.isdir(self.path):
            shutil.rmtree(self.path)
        os.mkdir(self.path)

        self.semaphore = True
        self.start_time = self.last_time = rospy.Time.now()
        while self.semaphore:
            pass
        self.images_taken = 0
        return 'finished_scan'
