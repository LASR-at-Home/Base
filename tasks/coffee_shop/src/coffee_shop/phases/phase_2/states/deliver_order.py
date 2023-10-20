#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from play_motion_msgs.msg import PlayMotionGoal
import json

class DeliverOrder(smach.State):
    def __init__(self, context):
        smach.StateMachine.__init__(self, outcomes=['done'])
        self.context = context

    def execute(self, userdata):
        self.context.voice_controller.sync_tts("I am going to deliver the order")
        location = rospy.get_param(f"/tables/{self.context.current_table}/location")
        position = location["position"]
        orientation = location["orientation"]
        #self.context.base_controller.sync_to_radius(self.context.tables[self.context.current_table]["people"][0][0], self.context.tables[self.context.current_table]["people"][0][1], 0.5)
        #self.context.base_controller.sync_face_to(self.context.tables[self.context.current_table]["people"][0][0], self.context.tables[self.context.current_table]["people"][0][1])
        self.context.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        self.context.base_controller.rotate(np.pi)
        pm_goal = PlayMotionGoal(motion_name="load_unload", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)
        self.context.voice_controller.async_tts("I'll give you some time to unload the order...")
        rospy.sleep(rospy.Duration(10.0))
        pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)
        return 'done'
