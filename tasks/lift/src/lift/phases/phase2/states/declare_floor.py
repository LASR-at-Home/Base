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
        # Listen to person:
        resp = self.listen()
        # Response in intent can either be yes or no.
        # Making sure that the response belongs to "affirm", not any other intent:
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

        # from lab
        # self.default.voice.speak(" Is the button selected?")
        # self.default.voice.speak("Please answer yes or no.")
        # rospy.sleep(1)
        # if RASA:
        #     answer = self.affirm()
        #     print("Answer from Speech: ", answer)
        #     #rasa get answer:
        #
        # else:
        #     req = AudioAndTextInteractionRequest()
        #     req.action = "BUTTON_PRESSED"
        #     req.subaction = "confirm_button"
        #     req.query_text = "SOUND:PLAYING:PLEASE"
        #     resp = self.default.speech(req)
        #     answer = resp.result

        # self.default.voice.speak("I got your answer.")
        # # get the answer
        # if answer == 'yes':
        #     self.default.voice.speak("Great! Thank you for pressing the button!")
        #     self.default.controllers.torso_controller.sync_reach_to(0.2)
        #     return 'success'
        # else:
        #     self.default.voice.speak("I will wait more")
        #     rospy.sleep(1)
        #     return 'failed'


