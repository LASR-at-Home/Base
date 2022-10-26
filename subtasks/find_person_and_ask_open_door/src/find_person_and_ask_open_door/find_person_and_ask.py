#!/usr/bin/env python3
import rospy
from tiago_controllers.base_controller import BaseController
from tiago_controllers.head_controller import HeadController
from lasr_voice.voice import Voice
from tiago_controllers.helpers import get_pose_from_param
from lasr_object_detection_yolo.person_detection import detect_person


# TODO: spin 180 degrees if no people found
# TODO: aim head at person

HORIZONTAL = 0.8
VERTICAL = 0.3
class FindPersonAndAsk:
    def __init__(self):
        self.base_controller = BaseController()
        self.head_controller = HeadController()
        self.head_controller.sync_reach_to(0, 0)  # start by centering the head
        self.voice = Voice()
        self.search_points = [(-1*HORIZONTAL, VERTICAL),
                              (0, VERTICAL),
                              (HORIZONTAL, VERTICAL),
                              (HORIZONTAL, 0),
                              (0, 0),
                              (-1*HORIZONTAL, 0),
                              (-1*HORIZONTAL, -1*VERTICAL),
                              (0, -1*VERTICAL),
                              (HORIZONTAL, -1*VERTICAL),
                              (0, 0)]

    def search_for_person(self):
        for point in self.search_points:
            self.head_controller.sync_reach_to(point[0], point[1])
            people = detect_person()
            if people:
                return people[0]
        return None

    def main(self):
        # door_pos = get_pose_from_param('/door_simu')
        door_pos = get_pose_from_param('/door')
        self.base_controller.sync_to_pose(door_pos)
        closest_person = self.search_for_person()
        if closest_person:
            self.voice.sync_tts("Can you please open the door for me?")
        else:
            self.voice.sync_tts("I can't see anyone!")


if __name__ == '__main__':
    rospy.init_node("find_person_and_ask_open_door_node", anonymous=True)
    find_and_ask = FindPersonAndAsk()
    find_and_ask.main()
