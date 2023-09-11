#!/usr/bin/env python3
import smach
import rospy
from collections import Counter
from play_motion_msgs.msg import PlayMotionGoal

class MakeOrder(smach.State):

    def __init__(self, context):
        smach.StateMachine.__init__(self, outcomes=['done'])
        self.context = context

    def execute(self, userdata):
        order = self.context.tables[self.context.current_table]["order"]
        order_string = ', '.join([f"{count} {self.context.target_object_remappings[item] if count == 1 else self.context.target_object_remappings[item]+'s'}" for item, count in Counter(order).items()]).replace(', ', ', and ', len(order)-2)
        self.context.voice_controller.sync_tts(f"Please get me {order_string}")
        self.context.voice_controller.sync_tts(f"Say 'finished' when you are ready for me to check the contents of the order")
        self.context.stop_head_manager("head_manager")
        pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)
        pm_goal = PlayMotionGoal(motion_name="check_table", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)
        return 'done'