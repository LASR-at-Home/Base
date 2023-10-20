#!/usr/bin/env python3
import smach
import rospy
from lift.defaults import TEST, PLOT_SHOW, PLOT_SAVE, DEBUG_PATH, DEBUG, RASA
from speech_helper import listen, affirm, hear_wait, get_people_number, get_floor
import json


class CheckAvailableExit(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success', 'failed', 'wait'])
        self.default = default

    def execute(self, userdata):
        floor = rospy.get_param("/floor/number")
        if floor == 0:
            floor = 1

        self.default.controllers.head_controller.look_straight()

        self.default.voice.speak("I really really want to go to the floor {}.".format(floor))
        answer = "no"
        if RASA:
            self.default.voice.speak("What floor is this?")
            try:
                floor_number = get_floor(self.default)
            except:
                floor_number = 3

            if (floor_number == floor):
                answer = "yes"

        new_answer = "no"
        if answer == "yes":
            self.default.voice.speak("Great! Let's finally exit to floor {}.".format(floor))
            self.default.controllers.torso_controller.sync_reach_to(0.2)
            return 'success'
        else:
            self.default.voice.speak("Does anyone want to exit on this floor?")
            if RASA:
                try:
                    new_answer = affirm(self.default)
                except:
                    new_answer = "yes"

            if new_answer == "yes":
                self.default.voice.speak("Great! Let's see how this will happen.")
                self.default.controllers.torso_controller.sync_reach_to(0.2)
                return 'wait'

            self.default.voice.speak("We can continue this lift journey")
            self.default.controllers.torso_controller.sync_reach_to(0.2)
            return 'failed'
