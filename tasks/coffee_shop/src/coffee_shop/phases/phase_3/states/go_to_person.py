#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Point

class GoToPerson(smach.State):
    def __init__(self, base_controller, voice_controller):
        smach.State.__init__(self, outcomes=['done'])
        self.base_controller = base_controller
        self.voice_controller = voice_controller

    def execute(self, userdata):
        self.voice_controller.sync_tts("I am going to approach the customer")
        pose = rospy.get_param("/person/position")
        self.base_controller.sync_to_radius(pose[0], pose[1], radius = 2.5)
        self.base_controller.sync_face_to(pose[0], pose[1])
        return 'done'