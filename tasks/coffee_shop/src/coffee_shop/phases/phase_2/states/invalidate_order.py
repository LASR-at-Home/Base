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
        previous_given_order = rospy.get_param(f"/tables/{rospy.get_param('current_table')}/previous_given_order", None)
        given_order = rospy.get_param(f"/tables/{rospy.get_param('current_table')}/given_order")

        if previous_given_order == given_order:
            rospy.sleep(rospy.Duration(5.0))
            return 'done'

        missing_items = list((Counter(order) - Counter(given_order)).elements())
        missing_items_string = ', '.join([f"{count} {item if count == 1 else item+'s'}" for item, count in Counter(missing_items).items()]).replace(', ', ', and ', len(missing_items) - 2)
        invalid_items = list((Counter(given_order) - Counter(order)).elements())
        invalid_items_string = ', '.join([f"{count} {item if count == 1 else item+'s'}" for item, count in Counter(invalid_items).items()]).replace(', ', ', and ', len(invalid_items) - 2)
        print(order, given_order)

        if not len(invalid_items):
            self.voice_controller.sync_tts(f"You didn't give me {missing_items_string} which I asked for. Please correct the order.")
        elif not len(missing_items):
            self.voice_controller.sync_tts(f"You have given me {invalid_items_string} which I didn't ask for. Please correct the order.")
        else:
            self.voice_controller.sync_tts(f"You have given me {invalid_items_string} which I didn't ask for, and didn't give me {missing_items_string} which I asked for. Please correct the order.")
        rospy.sleep(rospy.Duration(5.0))
        rospy.set_param(f"/tables/{rospy.get_param('current_table')}/previous_given_order", given_order)
        return 'done'