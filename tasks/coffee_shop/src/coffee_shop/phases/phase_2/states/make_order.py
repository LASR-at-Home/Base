#!/usr/bin/env python3
import smach
import rospy
from collections import Counter

class MakeOrder(smach.State):

    def __init__(self, context):
        smach.StateMachine.__init__(self, outcomes=['done'])
        self.context = context

    def execute(self, userdata):
        order = rospy.get_param(f"/tables/{rospy.get_param('/current_table')}/order")
        order_string = ', '.join([f"{count} {item if count == 1 else item+'s'}" for item, count in Counter(order).items()]).replace(', ', ', and ', len(order)-2)
        self.context.voice_controller.sync_tts(f"Please get me {order_string}")
        self.context.voice_controller.sync_tts(f"Say 'all done' when you are ready for me to check the contents of the order")
        rospy.sleep(rospy.Duration(5.0))
        return 'done'