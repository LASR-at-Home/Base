#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal


class DeliverOrder(smach.State):
    def __init__(self, base_controller, voice_controller, pm, speech):
        smach.StateMachine.__init__(self, outcomes=['done'])
        self.base_controller = base_controller
        self.voice_controller = voice_controller
        self.play_motion_client = pm
        self.speech = speech

    def execute(self, userdata):
        self.voice_controller.sync_tts("I am going to deliver the order")
        location = rospy.get_param(f"/tables/{rospy.get_param('/current_table')}/location")
        position = location["position"]
        orientation = location["orientation"]
        self.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        self.base_controller.rotate(np.pi)
        pm_goal = PlayMotionGoal(motion_name="load_unload", skip_planning=True)
        self.play_motion_client.send_goal_and_wait(pm_goal)
        self.voice_controller.sync_tts(
            "Please unload the order and say `all done` when you are ready for me to deliver it.")
        while True:
            resp = self.speech()
            if [entity["wake"]["value"] for entity in resp["entities"]]:
                break
        return 'done'