#!/usr/bin/env python3
import smach
import rospy
from interaction_module.srv import AudioAndTextInteraction, AudioAndTextInteractionRequest, \
    AudioAndTextInteractionResponse
import json
from lift.defaults import TEST, PLOT_SHOW, PLOT_SAVE, DEBUG_PATH, DEBUG, RASA
from speech_helper import listen, affirm

class DeclareFloor(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.default = default

    def execute(self, userdata):
        floor = rospy.get_param("/floor/number")
        rospy.set_param("/in_lift/status", True)
        if floor == 0:
            floor = 1

        # maybe add the counter here as well
        self.default.voice.speak("I would love to go to the floor {}.".format(floor))
        self.default.voice.speak("I don't see a button around, so I will assume the non-existent button is pressed.")

        return 'success'



