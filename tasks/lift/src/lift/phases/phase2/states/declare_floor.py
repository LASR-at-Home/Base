#!/usr/bin/env python3
import smach
import rospy
from interaction_module.srv import AudioAndTextInteraction, AudioAndTextInteractionRequest, \
    AudioAndTextInteractionResponse
import json
from lift.defaults import TEST, PLOT_SHOW, PLOT_SAVE, DEBUG_PATH, DEBUG, RASA

class DeclareFloor(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.default = default

    def listen(self):
        resp = self.default.speech()
        if not resp.success:
            self.default.voice.speak("Sorry, I didn't get that")
            return self.listen()
        resp = json.loads(resp.json_response)
        rospy.loginfo(resp)
        return resp

    def affirm(self):
        resp = self.listen()
        if resp['intent']['name'] != 'affirm':
            self.default.voice.speak("Sorry, I didn't get that, please say yes or no")
            return self.affirm()
        choices = resp["entities"].get("choice", None)
        if choices is None:
            self.default.voice.speak("Sorry, I didn't get that")
            return self.affirm()
        choice = choices[0]["value"].lower()
        if choice not in ["yes", "no"]:
            self.default.voice.speak("Sorry, I didn't get that")
            return self.affirm()
        return choice 


    def execute(self, userdata):
        floor = rospy.get_param("/floor/number")
        rospy.set_param("/in_lift/status", True)
        if floor == 0:
            floor = 1

        # maybe add the counter here as well
        self.default.voice.speak("I would love to go to the floor {}.".format(floor))
        self.default.voice.speak("I don't see a button around, so I will assume the non-existent button is pressed.")

        return 'success'



