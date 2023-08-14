#!/usr/bin/env python3
import smach
import rospy

class DeliverOrder(smach.State):
    def __init__(self, base_controller, voice_controller):
        smach.StateMachine.__init__(self, outcomes=['done'])
        self.base_controller = base_controller
        self.voice_controller = voice_controller

    def execute(self, userdata):
        return 'done'