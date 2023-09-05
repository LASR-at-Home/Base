#!/usr/bin/env python3
import smach
from tiago_controllers.controllers.look_at import LookAt
from tiago_controllers.controllers.base_controller import CmdVelController
from lasr_object_detection_yolo.detect_objects_v8 import detect_objects
from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction
import rospy
import actionlib

HORIZONTAL = 0.8
VERTICAL = 0.3

class FacePerson(smach.State):
    def __init__(self, controllers, voice, yolo, cmd):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.controllers = controllers
        self.voice = voice
        self.yolo = yolo
        self.cmd_vel = cmd

        self.search_points = [(-1 * HORIZONTAL, VERTICAL),
                              (0, VERTICAL),
                              (HORIZONTAL, VERTICAL),
                              (HORIZONTAL, 0),
                              (0, 0),
                              (-1 * HORIZONTAL, 0),
                              (-1 * HORIZONTAL, -1 * VERTICAL),
                              (0, -1 * VERTICAL),
                              (HORIZONTAL, -1 * VERTICAL),
                              (0, 0)]

        self.client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)

    def search_for_person(self):
        for point in self.search_points:
            self.controllers.head_controller.sync_reach_to(point[0], point[1])
            people = detect_objects(["person"])
            if people:
                return people[0], point[0]
        return None

    def play_motion_goal(self, motion_name='home'):
        self.client.wait_for_server()
        rospy.sleep(0.5)
        goal = PlayMotionGoal()

        goal.motion_name = motion_name
        self.client.send_goal(goal)
        result = self.client.wait_for_result()
        state = self.client.get_state()
        return result, state


    def execute(self, userdata):
        turns = 4
        for i in range(turns):
            try:
                closest_person, head_rot = self.search_for_person()
                # self.play_motion_goal('home')
                self.controllers.torso_controller.sync_reach_to(0.25)
                if closest_person:
                    look_at = LookAt(self.controllers.head_controller, self.controllers.base_controller, self.cmd_vel, "person")
                    look_at.look_at(closest_person.xywh, head_rot)
                    return 'success'
            except TypeError:
                self.cmd_vel.rotate(60, 360/turns, True)

        self.voice.speak("I can't see anyone!")
        # self.controllers.base_controller.sync_face_to(pose[0], pose[1])
        return 'failed'