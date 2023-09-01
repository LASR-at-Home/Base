#!/usr/bin/env python3
import smach
from tiago_controllers.controllers.look_at import LookAt
from tiago_controllers.controllers.base_controller import CmdVelController
from lasr_object_detection_yolov8.detect_objects import detect_objects
import rospy

HORIZONTAL = 0.8
VERTICAL = 0.3

class FacePerson(smach.State):
    def __init__(self, controllers, voice, yolo):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.controllers = controllers
        self.voice = voice
        self.yolo = yolo

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

    def search_for_person(self):
        for point in self.search_points:
            self.controllers.head_controller.sync_reach_to(point[0], point[1])
            people = detect_objects(["person"])
            if people:
                return people[0], point[0]
        return None

    def execute(self, userdata):
        turns = 4
        for i in range(turns):
            try:
                closest_person, head_rot = self.search_for_person()
                if closest_person:
                    look_at = LookAt(self.controllers.head_controller, self.controllers.base_controller, self.cmd_vel, "person")
                    look_at.look_at(closest_person.xywh, head_rot)
                    return 'success'
            except TypeError:
                self.cmd_vel.rotate(60, 360/turns, True)

        self.voice.speak("I can't see anyone!")
        # self.controllers.base_controller.sync_face_to(pose[0], pose[1])
        return 'failed'