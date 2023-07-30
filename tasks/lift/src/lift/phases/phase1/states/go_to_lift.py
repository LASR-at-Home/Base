#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from tiago_controllers.helpers.pose_helpers import get_pose_from_param

class GoToLift(smach.State):
    def __init__(self, controllers, voice):
        smach.State.__init__(self, outcomes=['success'])

        self.controllers = controllers
        self.voice = voice

    def execute(self, userdata):
        result = self.controllers.base_controller.sync_to_pose(get_pose_from_param('/lift/pose'))

        return 'success'