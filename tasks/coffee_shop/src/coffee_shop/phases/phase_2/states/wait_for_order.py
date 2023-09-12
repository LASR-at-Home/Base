#!/usr/bin/env python3
import smach
import json
import rospy

class WaitForOrder(smach.State):

    def __init__(self, context):
        smach.StateMachine.__init__(self, outcomes=['done'])
        self.context = context

    def execute(self, userdata):
        resp = self.context.speech(True)
        while True:
            if not resp.success:
                continue
            resp = json.loads(resp.json_response)
            rospy.loginfo(resp)
            if "finished" in resp["text"].lower():
                break
            resp = self.context.speech(False)
        return 'done'
