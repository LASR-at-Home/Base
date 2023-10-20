#!/usr/bin/env python3
import smach
from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

class GoCloserToPerson(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['done'])
        self.context = context

    def execute(self, userdata):
        self.context.voice_controller.async_tts("I think there is a customer waiting. I will go and investigate.")
        location = rospy.get_param("/wait/approach1")
        position, orientation = location["position"], location["orientation"]
        self.context.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        return 'done'