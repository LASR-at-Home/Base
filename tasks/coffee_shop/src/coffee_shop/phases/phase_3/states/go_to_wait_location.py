#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

class GoToWaitLocation(smach.State):
    def __init__(self, base_controller, voice_controller):
        smach.State.__init__(self, outcomes=['done', 'not done'])
        self.base_controller = base_controller
        self.voice_controller = voice_controller
    def execute(self, userdata):
        self.voice_controller.sync_tts("I am going to the wait for a new customer")
        wait_location = rospy.get_param("/wait")
        position, orientation = wait_location["location"]["position"], wait_location["location"]["orientation"]
        self.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        tables = rospy.get_param("/tables")
        return 'not done' if len([table for table in tables.values() if table["status"] == "ready"]) else 'done'
