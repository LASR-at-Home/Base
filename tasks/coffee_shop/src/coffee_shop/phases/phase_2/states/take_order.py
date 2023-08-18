#!/usr/bin/env python3
import smach
import rospy
import random

MOCK_ORDERS = [
    ["cup"],
    ["cup", "cup"],
    ["cup", "cup", "cup"]
]

class TakeOrder(smach.State):
    
    def __init__(self, head_controller, voice_controller):
        smach.State.__init__(self, outcomes=['done'])
        self.head_controller = head_controller
        self.voice_controller = voice_controller

    def execute(self, userdata):
        self.voice_controller.sync_tts("Can I please take your order?")
        mock = random.choice(MOCK_ORDERS)
        self.voice_controller.sync_tts(f"MOCK ORDER: {mock.join(' ')}")
        rospy.set_param(f"/tables/{rospy.get_param('/current_table')}/order", mock)
        return 'done'
