#!/usr/bin/env python3
import smach
import json
import rospy

class WaitForOrder(smach.State):

    def __init__(self, context):
        smach.StateMachine.__init__(self, outcomes=['done'])
        self.context = context

    def execute(self, userdata):
        while True:
            resp = self.context.speech(False)
            if not resp.success:
                continue
            resp = json.loads(resp.json_response)
            rospy.loginfo(resp)
            if resp["intent"]["name"] == "affirm" and resp["text"].lower() == "done":
                break
        return 'done'
