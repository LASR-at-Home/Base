#!/usr/bin/env python3
import smach
import rospy

class GoToCounter(smach.State):
    
    def __init__(self, head_controller):
        smach.State.__init__(self, outcomes=['done'])
        self.head_controller = head_controller

    def execute(self, userdata):
        return 'done'
