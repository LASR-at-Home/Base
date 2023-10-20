#!/usr/bin/env python3
import smach
import rospy
from collections import Counter
from play_motion_msgs.msg import PlayMotionGoal
from std_msgs.msg import String

class MakeOrder(smach.State):

    def __init__(self, context):
        smach.StateMachine.__init__(self, outcomes=['done'])
        self.context = context
        self.tablet_pub = rospy.Publisher("/tablet/screen", String, queue_size=10) 

    def execute(self, userdata):
        self.context.stop_head_manager("head_manager")
        order = self.context.tables[self.context.current_table]["order"]
        order_string = ', '.join([f"{count} {self.context.target_object_remappings[item] if count == 1 else self.context.target_object_remappings[item]+'s'}" for item, count in Counter(order).items()]).replace(', ', ', and ', len(order)-2)
        self.context.voice_controller.sync_tts(f"Please get me {order_string}. Make sure you place the tray clearly in the view of the robot, please")
        return 'done'
