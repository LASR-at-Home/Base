#!/usr/bin/env python3

import rospy
import smach
from geometry_msgs.msg import Pose
from tiago_controllers.helpers.pose_helpers import get_pose_from_param


class GoToFinish(smach.State):
    def __init__(self, controllers, voice):
        smach.State.__init__(self, outcomes=['success'])
        self.controllers = controllers
        self.voice = voice

    def execute(self, userdata):
        rospy.loginfo('Executing state GoToFinish')
        pos = get_pose_from_param('/door')
        self.controllers.base_controller.sync_to_pose(pos)
        p = Pose()
        p.position.x = 0.0
        p.position.y =  0.0
        p.orientation.w = 1
        success = self.controllers.base_controller.sync_to_pose(p)
        if not success:
            return 'failed'
        return 'success'

if __name__ == '__main__':
    rospy.init_node("go_to_finish", anonymous=True)
