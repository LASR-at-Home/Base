#!/usr/bin/env python3
import smach
import rospy
from collections import Counter

class InvalidateOrder(smach.State):
    def __init__(self, voice_controller):
        smach.State.__init__(self, outcomes=['done'])
        self.voice_controller = voice_controller

    def execute(self, userdata):
        order = rospy.get_param(f"/tables/{rospy.get_param('current_table')}/order")
        given_order = rospy.get_param(f"/tables/{rospy.get_param('current_table')}/given_order")
        missing_items = list(set(order) - set(given_order))
        missing_items_string = ', '.join([f"{count} {item if count == 1 else item+'s'}" for item, count in Counter(missing_items).items()]).replace(', ', ', and ', len(missing_items) - 2)
        invalid_items = list(set(given_order) - set(order))
        invalid_items_string = ', '.join([f"{count} {item if count == 1 else item+'s'}" for item, count in Counter(invalid_items).items()]).replace(', ', ', and ', len(invalid_items) - 2)
        self.voice_controller.sync_tts(f"You have given me {invalid_items_string} which I didn't ask for, and didn't give me {missing_items_string} which I did ask for. Please correct the order.")
        return 'done'
