#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion


class GoToTable(smach.State):
    
    def __init__(self, base_controller):
        smach.State.__init__(self, outcomes=['done'])
        self.base_controller = base_controller

    def execute(self, userdata):
        position = rospy.get_param(f"/tables/{rospy.get_param('/current_table')}/position")
        orientation = rospy.get_param(f"/tables/{rospy.get_param('/current_table')}/orientation")
        self.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        return 'done'