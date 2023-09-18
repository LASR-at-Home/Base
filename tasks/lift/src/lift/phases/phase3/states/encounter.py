#!/usr/bin/env python3
import os
import smach
from pal_startup_msgs.srv import StartupStart, StartupStop
import rosservice
import rospy
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
import numpy as np
import ros_numpy as rnp
from object_interest_tracking.srv import  Tdr
from geometry_msgs.msg import Pose

HORIZONTAL = 0.8
VERTICAL = 0.3
class Encounter(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success'])
        
        self.default = default

        # stop head manager
        if "/pal_startup_control/stop" in rosservice.get_service_list():
            self.stop_head_manager = rospy.ServiceProxy("/pal_startup_control/stop", StartupStop)
            self.start_head_manager = rospy.ServiceProxy("/pal_startup_control/start", StartupStart)

    def execute(self, userdata):
        # result = self.controllers.base_controller.sync_to_pose(get_pose_from_param('/lift/pose'))
        # get the point from zoe

        self.stop_head_manager()

        self.voice.speak("Hi")
        rospy.wait_for_service('/v2')
        rospy.ServiceProxy('/v2', Tdr)()
        self.default.voice.speak("Hi mate, nice to meet you, you Look great today. I have just arrived at the second floor.")

        return 'success'


if __name__ == '__main__':
    rospy.init_node("encounter", anonymous=True)
