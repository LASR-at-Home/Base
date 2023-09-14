#!/usr/bin/env python3
import smach
import json
import rospy
from play_motion_msgs.msg import PlayMotionGoal


class WaitForOrder(smach.State):

    def __init__(self, context):
        smach.StateMachine.__init__(self, outcomes=['done'])
        self.context = context

    def execute(self, userdata):
        pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)
        resp = self.context.speech(True)
        resp = json.loads(resp.json_response)
        while True:
            rospy.loginfo(resp)
            if "finish" in resp["text"].lower():
                break
            resp = self.context.speech(False)
            resp = json.loads(resp.json_response)
        return 'done'
