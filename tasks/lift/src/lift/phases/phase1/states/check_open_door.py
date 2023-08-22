#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from choosing_wait_position.final_lift_key_point.predict_pos import make_prediction
from narrow_space_navigation.waypoints import *
from tiago_controllers.controllers.controllers import Controllers
from sensor_msgs.msg import LaserScan
from PIL import Image
import numpy as np
MEAN_DISTANCE_THRESHOLD = 1.5

class CheckOpenDoor(smach.State):
    def __init__(self, controllers, voice):
        smach.State.__init__(self, outcomes=['success', 'failed'])

        self.controllers = controllers
        self.voice = voice

    def execute(self, userdata):
        # check for open door
        laser_scan = rospy.wait_for_message("/scan", LaserScan)
        filtered_ranges = laser_scan.ranges[len(laser_scan.ranges) // 3: 2 * len(laser_scan.ranges) // 3]
        mean_distance = np.nanmean(filtered_ranges)
        print('mean distance =====> ', mean_distance)
        if mean_distance < MEAN_DISTANCE_THRESHOLD or mean_distance == np.inf or mean_distance == np.nan:
            # if no, go back to scan and choose elevator
            print(mean_distance, 'is less than', MEAN_DISTANCE_THRESHOLD)
            return 'failed'
        else:
            return 'success'


