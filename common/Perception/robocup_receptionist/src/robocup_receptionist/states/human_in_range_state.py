#!/usr/bin/env python3
import rospy
import smach
import rospy
import numpy as np

from cv_bridge3 import CvBridge
from robocup_receptionist.utils.servers_clients import detect_person
from dialogflow_speech.utils import talk
import random

class WaitForGuestState(smach.State):
    
    def __init__(self, base_controller):
        smach.State.__init__(self, outcomes=['human_in_range', 'not_human_in_range'], output_keys=["person_bb"])
        self.bridge = CvBridge()
        self.base_controller = base_controller
        self.distance_maximum = 2.0
        self.distance_minimum = 1.5
        self.say_come_closer = False

    def execute(self, userdata):
        detections = detect_person()
        if not detections:
            return 'not_human_in_range'

        if detections:
            robot_pos = np.array(self.base_controller.get_current_pose()[:2])
            centroids = [np.array([d.centroid.point.x, d.centroid.point.y]) for d in detections]
            distances = [np.linalg.norm(robot_pos - c) for c in centroids]
            n = np.argmin(distances)
            if distances[n] < self.distance_maximum:
                userdata.person_bb = detections[n].xywh
                talk(random.choice(["Greetings", "Hello", "Hi", "Welcome buddy!", "What's up buttercup!", "Welcome"]))
                talk("Please stay there for a moment.")
                self.base_controller.sync_face_to(detections[n].centroid.point.x, detections[n].centroid.point.y)
                return 'human_in_range'
            # elif distances[n] < self.distance_minimum:
            #     talk("you're standing too close, please go a step backwards")
            #     return 'not_human_in_range'
            else:
                if not self.say_come_closer:
                    talk("Please come a bit closer.")
                    self.say_come_closer = True
                return 'not_human_in_range'