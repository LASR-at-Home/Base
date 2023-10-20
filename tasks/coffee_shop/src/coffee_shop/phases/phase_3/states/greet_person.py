#!/usr/bin/env python3
import smach
from play_motion_msgs.msg import PlayMotionGoal
from control_msgs.msg import PointHeadGoal
from geometry_msgs.msg import Point


class GreetPerson(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['done'])
        self.context = context

    def execute(self, userdata):
        self.context.stop_head_manager("head_manager")
        """
        ph_goal = PointHeadGoal()
        ph_goal.max_velocity = 1.0
        ph_goal.pointing_frame = 'head_2_link'
        ph_goal.pointing_axis = Point(1.0, 0.0, 0.0)
        ph_goal.target.header.frame_id = 'map'
        ph_goal.target.point = Point(*self.context.new_customer_pose)
        self.context.point_head_client.send_goal_and_wait(ph_goal)
        """
        self.context.voice_controller.sync_tts("Hi there! My name is TIAGO. Please follow me, I'll guide you to a table.")
        pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)
        self.context.start_head_manager("head_manager", '')
        return 'done'