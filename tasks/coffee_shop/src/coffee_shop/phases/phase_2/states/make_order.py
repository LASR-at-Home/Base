#!/usr/bin/env python3
import smach
import rospy

class MakeOrder(smach.State):

    def __init__(self, voice_controller):
        smach.StateMachine.__init__(self, outcomes=['done'])
        self.voice_controller = voice_controller

    def execute(self, userdata):
        return 'done'
