#!/usr/bin/env python3
# coding=utf-8
import os
import shutil
import smach
import rospy
from sensor_msgs.msg import Image
import rospkg
from cv_bridge3 import CvBridge
from cv_bridge3 import cv2

import random
import string


IMAGE_NUMBER = 10
SLEEP_TIME = 0.8
MAX_DURATION = IMAGE_NUMBER*SLEEP_TIME + 2

class CreateDatasetState(smach.State):
    '''
    Creates a dataset of a person and trains a model to redetect that individual.
    To be able to train, the dataset must contain at least 2 people
    '''
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['finished_scan'],
                             output_keys=['dataset_path']
        )
        self.name = ""
        self.path = ""
        self.dir_path = self.path = os.path.join(rospkg.RosPack().get_path('create_dataset'),  'dataset')
        self.bridge = CvBridge()
        self.images_taken = 0
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
        userdata.dataset_path = self.path = os.path.join(rospkg.RosPack().get_path('create_dataset'),  'dataset', rand)

        if os.path.isdir(self.path):
            shutil.rmtree(self.path)
        os.mkdir(self.path)

        self.semaphore = True
        self.start_time = self.last_time = rospy.Time.now()
        while self.semaphore:
            pass
        self.images_taken = 0
        return 'finished_scan'


if __name__ == '__main__':
   rospy.init_node('smach_example_state_machine')
   sm = smach.StateMachine(outcomes=['success'])
   with sm:
      smach.StateMachine.add('CREATE_DATASET', CreateDatasetState(),
                              transitions={'finished_scan':'success'},
                              remapping={'dataset_path':'dataset_path'})
   outcome = sm.execute()
   rospy.loginfo('I have completed execution with outcome: ')
   rospy.loginfo(outcome)