#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Point

class GoToPerson(smach.State):
    def __init__(self, base_controller):
        smach.State.__init__(self, outcomes=['done'])
        self.base_controller = base_controller

    def execute(self, userdata):
        pose = rospy.get_param("/person/position")
        self.base_controller.sync_to_radius(pose[0], pose[1], radius = 1.0)
        return 'done'