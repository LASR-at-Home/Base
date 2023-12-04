#!/usr/bin/env python3

import rospy
import smach
from tiago_controllers.controllers import Controllers

class GoToLocation(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['location'])
        self.controllers = Controllers()

    def execute(self, userdata):
        try:
            status = self.controllers.base_controller.sync_to_pose(userdata.location)
            if status:
                return 'succeeded'
            return 'failed'
        except rospy.ERROR as e:
            rospy.logwarn(f"Unable to go to location. {userdata.location} -> ({str(e)})")
            return 'failed'