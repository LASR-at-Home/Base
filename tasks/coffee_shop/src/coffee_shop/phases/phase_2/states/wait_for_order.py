#!/usr/bin/env python3
import smach
import json
import rospy

class WaitForOrder(smach.State):

    def __init__(self, speech):
        smach.StateMachine.__init__(self, outcomes=['done'])
        self.speech = speech

    def execute(self, userdata):
        while True:
            resp = self.speech()
            if not resp.success:
                continue
            resp = json.loads(resp.json_response)
            rospy.loginfo(resp)
            if resp["intent"]["name"] != "wake_word":
                continue
            if resp["entities"].get("wake", None):
                break
        return 'done'