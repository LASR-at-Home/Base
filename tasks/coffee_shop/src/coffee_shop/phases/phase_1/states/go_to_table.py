#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np

class GoToTable(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['done'])
        self.context = context

    def execute(self, userdata):
        location = rospy.get_param(f"/tables/{self.context.current_table}/location")
        self.context.base_controller.sync_to_pose(Pose(position=Point(**location["position"]), orientation=Quaternion(**location["orientation"])))
        return 'done'
