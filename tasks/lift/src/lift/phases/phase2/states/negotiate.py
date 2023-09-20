#!/usr/bin/env python3
import smach
import rospy
import numpy as np
import json
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Empty
from tiago_controllers.helpers.nav_map_helpers import clear_costmap
from interaction_module.srv import AudioAndTextInteraction, AudioAndTextInteractionRequest, \
    AudioAndTextInteractionResponse
from tiago_controllers.helpers.nav_map_helpers import is_close_to_object, rank
from lift.defaults import TEST, PLOT_SHOW, PLOT_SAVE, DEBUG_PATH, DEBUG, RASA
from lasr_object_detection_yolo.detect_objects_v8 import detect_objects, perform_detection, debug
from sensor_msgs.msg import PointCloud2
from speech_helper import listen, affirm, hear_wait


class Negotiate(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.default = default

        self.head_motions = {
            'look_straight': self.default.controllers.head_controller.look_straight,
            'look_right': self.default.controllers.head_controller.look_right,
            'look_left': self.default.controllers.head_controller.look_left
        }

    def is_anyone_in_front_of_me(self):
        detections = 0
        for motion in self.head_motions:
            self.head_motions[motion]()
            rospy.sleep(1)
            pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
            polygon = rospy.get_param('/corners_lift')
            detections, im = perform_detection(self.default, pcl_msg, polygon, ['person'])
            if len(detections) > 0:
                self.default.controllers.head_controller.look_straight()
                return True
        return False

    def execute(self, userdata):
        self.default.voice.speak("Let's negotiate who is going out first")
        is_closer_to_door = not self.is_anyone_in_front_of_me()

        if is_closer_to_door:
            self.default.voice.speak("I am the closest to the door so I have to exit first")
            # clear costmap
            clear_costmap()
            # go to centre waiting area
            self.default.voice.speak("I will wait by the lift for you.")
            # TODO: here maybe switch to the wait_centre
            res = self.default.controllers.base_controller.sync_to_pose(get_pose_from_param('/wait/pose'))
        else:
            self.default.voice.speak("I am not the closest to the door.")
            self.default.voice.speak("I will wait for you to exit first")
            rospy.sleep(10)


        if is_closer_to_door:
            # clear costmap
            clear_costmap()
            # maybe take the lift info again
            self.default.voice.speak("Exiting the lift")

        return 'success'
