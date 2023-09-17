#!/usr/bin/env python3

import rospy
import smach
from geometry_msgs.msg import Pose
from tiago_controllers.helpers.pose_helpers import get_pose_from_param


class GoToFinish(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success'])
        self.default = default

    def execute(self, userdata):
        rospy.loginfo('Executing state GoToFinish')
        self.default.controllers.base_controller.sync_to_pose(get_pose_from_param('/door/pose'))
        p = Pose()
        p.position.x = 0.0
        p.position.y =  0.0
        p.orientation.w = 1
        success = self.default.controllers.base_controller.sync_to_pose(p)
        if not success:
            return 'failed'
        return 'success'

if __name__ == '__main__':
    rospy.init_node("go_to_finish", anonymous=True)
