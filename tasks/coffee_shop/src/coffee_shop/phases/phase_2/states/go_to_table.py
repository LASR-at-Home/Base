#!/usr/bin/env python3
import smach
import rospy

class GoToTable(smach.State):
    def execute(self, userdata, outcomes='done'):
        return 'done'

