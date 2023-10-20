#!/usr/bin/env python3
import rospy
from tiago_controllers.controllers.base_controller import CmdVelController
# from tiago_controllers.head_controller import HeadController
from tiago_controllers.controllers.look_at import LookAt
# from lasr_voice.voice import Voice
from tiago_controllers.helpers import get_pose_from_param
from lasr_object_detection_yolo.detect_objects import detect_objects
from tiago_controllers.controllers import Controllers

# TODO: make statemachine
# TODO: add simulation:= true/false in launch file

HORIZONTAL = 0.8
VERTICAL = 0.3


class FindPersonAndAsk:
    def __init__(self):
        self.controllers = Controllers()
        self.base_controller = self.controllers.base_controller
        self.head_controller = self.controllers.head_controller
        self.cmd_vel_controller = CmdVelController()

        self.head_controller.sync_reach_to(0, 0)  # start by centering the head
        # self.voice = Voice()
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
            self.head_controller.sync_reach_to(point[0], point[1])
            people = detect_objects(["person"], "coco")
            if people:
                return people[0], point[0]
        return None

    # ! State machine 1: go to place
    # ! State machine 2: search
    # ! State machine 3: turn
    # ! State machine 4: talk
    def main(self):
        # door_pos = get_pose_from_param('/door_simu')
        # door_pos = get_pose_from_param('/door')
        # self.base_controller.sync_to_pose(door_pos)
        cmd_vel = CmdVelController()
        turns = 3
        for i in range(turns):
            try:
                closest_person, head_rot = self.search_for_person()
                if closest_person:
                    print("FOUND PERSON")
                    look_at = LookAt(self.head_controller, self.base_controller, self.cmd_vel_controller, "person")
                    look_at.look_at(closest_person.xywh, head_rot)
                    print("CAN YOU PLEASE OPEN THE DOOR")
                    self.voice.sync_tts("Can you please open the door for me?")
                    return
            except TypeError:
                cmd_vel.rotate(60, 360/turns, True)

        print("I CANT SEE ANYONE")
        self.voice.sync_tts("I can't see anyone!")


if __name__ == '__main__':
    rospy.init_node("find_person_and_ask_open_door_node", anonymous=True)
    find_and_ask = FindPersonAndAsk()
    find_and_ask.main()
