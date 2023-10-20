#!/usr/bin/env python3
import smach
from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

class GoToPerson(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['done'])
        self.context = context

    def execute(self, userdata):
        location = rospy.get_param("/wait/approach2")
        position, orientation = location["position"], location["orientation"]
        self.context.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        if self.context.new_customer_pose:
            self.context.base_controller.sync_face_to(self.context.new_customer_pose[0], self.context.new_customer_pose[1])
        return 'done'