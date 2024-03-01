#!/usr/bin/env python3
import smach, rospy
from tiago_controllers.controllers.look_at import LookAt
from lasr_object_detection_yolo.detect_objects_v8 import detect_objects, perform_detection, debug
from sensor_msgs.msg import PointCloud2

HORIZONTAL = 0.8
VERTICAL = 0.3

class FacePerson(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.default = default

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
            self.default.controllers.head_controller.sync_reach_to(point[0], point[1])
            people = detect_objects(["person"])
            if people:
                return people[0], point[0]
        return None


    def execute(self, userdata):
        turns = 4
        for i in range(turns):
            try:
                closest_person, head_rot = self.search_for_person()
                self.default.controllers.torso_controller.sync_reach_to(0.25)
                if closest_person:
                    look_at = LookAt(self.default.controllers.head_controller, self.default.controllers.base_controller, self.default.cmd, "person")
                    look_at.look_at(closest_person.xywh, head_rot)
                    return 'success'
            except TypeError:
                self.default.cmd.rotate(60, 360/turns, True)

        self.default.voice.speak("I can't see anyone!")
        return 'failed'