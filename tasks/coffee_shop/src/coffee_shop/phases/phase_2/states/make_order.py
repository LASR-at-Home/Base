#!/usr/bin/env python3
import smach
import rospy
from collections import Counter

class MakeOrder(smach.State):

    def __init__(self, voice_controller):
        smach.StateMachine.__init__(self, outcomes=['done'])
        self.voice_controller = voice_controller

    def execute(self, userdata):
        order = rospy.get_param(f"/tables/{rospy.get_param('/current_table')}/order")
        order_string = ', '.join([f"{count} {item if count == 1 else item+'s'}" for item, count in Counter(order).items()]).replace(', ', ', and ', len(order)-2)
        self.voice_controller.sync_tts(f"Please get me {order_string}")
        return 'done'