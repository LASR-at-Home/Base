#!/usr/bin/env python3

import rospy
import smach
from geometry_msgs.msg import Pose
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from std_msgs.msg import Empty, Int16


class GoToFinish(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success'])
        self.default = default

    def execute(self, userdata):
        rospy.loginfo('Executing state GoToFinish')
        self.default.controllers.base_controller.sync_to_pose(get_pose_from_param('/starting/pose'))

        # self.default.datahub_stop_phase.publish(Int16(3))
        # self.default.datahub_stop_episode.publish(Empty())
        return 'success'

if __name__ == '__main__':
    rospy.init_node("go_to_finish", anonymous=True)
