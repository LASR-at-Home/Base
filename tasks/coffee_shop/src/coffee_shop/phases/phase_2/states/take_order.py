#!/usr/bin/env python3
import smach
import rospy

class TakeOrder(smach.State):
    
    def __init__(self, head_controller, voice_controller):
        smach.State.__init__(self, outcomes=['done'])
        self.head_controller = head_controller
        self.voice_controller = voice_controller

    def execute(self, userdata):
        return 'done'
