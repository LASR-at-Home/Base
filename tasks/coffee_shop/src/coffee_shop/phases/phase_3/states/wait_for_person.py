#!/usr/bin/env python3
import smach
import rospy

class WaitForPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
    def execute(self, userdata):
        return 'done'
