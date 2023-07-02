#!/usr/bin/env python3
import smach
import rospy

class GoToWaitLocation(smach.State):
    def __init__(self, base_controller):
        smach.State.__init__(self, outcomes=['done'])
        self.base_controller = base_controller
    def execute(self, userdata):
        return 'done'


