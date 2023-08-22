#!/usr/bin/env python3
import smach
import rospy
import random

MOCK_ORDER = ["cup", "cup"]
class TakeOrder(smach.State):
    
    def __init__(self, head_controller, voice_controller):
        smach.State.__init__(self, outcomes=['done'])
        self.head_controller = head_controller
        self.voice_controller = voice_controller

    def execute(self, userdata):
        self.voice_controller.sync_tts("Can I please take your order?")
        self.voice_controller.sync_tts(f"MOCK ORDER: {' '.join(MOCK_ORDER)}")
        rospy.set_param(f"/tables/{rospy.get_param('/current_table')}/order", MOCK_ORDER)
        return 'done'
