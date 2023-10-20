#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

class GoToCounter(smach.State):
    
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['done'])
        self.context = context

    def execute(self, userdata):
        self.context.voice_controller.async_tts("I am going to the counter to retrieve the order")
        position = rospy.get_param("counter/location/position")
        orientation = rospy.get_param("counter/location/orientation")
        self.context.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        return 'done'