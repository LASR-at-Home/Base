#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

class DeliverOrder(smach.State):
    def __init__(self, base_controller, voice_controller):
        smach.StateMachine.__init__(self, outcomes=['done'])
        self.base_controller = base_controller
        self.voice_controller = voice_controller

    def execute(self, userdata):
        self.voice_controller.sync_tts("I am going to deliver the order")
        location = rospy.get_param(f"/tables/{rospy.get_param('/current_table')}/location")
        position = location["position"]
        orientation = location["orientation"]
        self.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        return 'done'