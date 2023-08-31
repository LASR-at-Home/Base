#!/usr/bin/env python3
import smach
import rospy

class HandOverOrder(smach.State):

    def __init__(self, context):
        smach.StateMachine.__init__(self, outcomes=['done'])
        self.context = context

    def execute(self, userdata):
        self.context.voice_controller.sync_tts("I don't actually have your order! Whoops...")
        return 'done'