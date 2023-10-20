#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

class GoToWaitLocation(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['done', 'not done'])
        self.context = context
        self.done = False

    def execute(self, userdata):
        wait_location = rospy.get_param("/wait")
        position, orientation = wait_location["location"]["position"], wait_location["location"]["orientation"]
        self.context.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        if not self.done:
            self.done = True
            return 'not done'
        else:
            return 'done'