#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion


class GoToTable(smach.State):
    
    def __init__(self, base_controller):
        smach.State.__init__(self, outcomes=['done'])
        self.base_controller = base_controller

    def execute(self, userdata):
        table, data = [(table, data) for table, data in rospy.get_param("/tables").items() if data["status"] == "needs_serving"][0]
        position = data["location"]["position"]
        orientation = data["location"]["orientation"]
        self.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        rospy.set_param("/current_table", table)
        return 'done'