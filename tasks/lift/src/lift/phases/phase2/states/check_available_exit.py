#!/usr/bin/env python3
import smach
import rospy
from interaction_module.srv import AudioAndTextInteraction, AudioAndTextInteractionRequest, \
    AudioAndTextInteractionResponse
from lift.defaults import TEST, PLOT_SHOW, PLOT_SAVE, DEBUG_PATH, DEBUG, RASA
import json
class CheckAvailableExit(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success', 'failed', 'wait'])
        self.default = default
    def listen(self):
        resp = self.default.speech()
        print("Resp success: ", resp.success)
        if not resp.success:
            self.default.voice.speak("Sorry, I didn't get that")
            return self.listen()
        resp = json.loads(resp.json_response)
        rospy.loginfo(resp)
        return resp

    def affirm(self):
        #Listen to person:
        resp = self.listen()
        #Response in intent can either be yes or no.
        #Making sure that the response belongs to "affirm", not any other intent: 
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

    def get_floor(self):
        resp = self.listen()
        if resp["intent"]["name"] != "check_floor_number":
            self.default.voice.speak("Sorry, I misheard you")
            return self.get_floor()
        floor = resp["entities"].get("floor",[])
        if not floor: 
            self.default.voice.speak("Sorry, I can't figure out that we're talking about floors")
            return self.get_floor()
        floor_number = int(floor[0]["value"])        
        self.default.voice.speak("I heard that we are on floor {}".format(floor_number))
        return floor_number

    def execute(self, userdata):
        floor = rospy.get_param("/floor/number")
        if floor == 0:
            floor = 1

        # ensure the head is straight
        self.default.controllers.head_controller.look_straight()

        self.default.voice.speak("I really really want to go to the floor {}.".format(floor))
        answer = "no"
        if RASA:
            self.default.voice.speak("What floor is this?")
            try:
                floor_number = self.get_floor()
            except:
                floor_number = 3

            if (floor_number==floor):
                answer = "yes"
        else:
            self.default.voice.speak("Is this the floor {}?".format(floor))
            # get the answer
            req = AudioAndTextInteractionRequest()
            req.action = "BUTTON_PRESSED"
            req.subaction = "confirm_button"
            req.query_text = "SOUND:PLAYING:PLEASE"
            resp = self.default.speech(req)
            rospy.loginfo("The response of asking the people in check available exit is {}".format(resp.result))
            answer = resp.result


        

        # get the answer
        new_answer = "no"
        if answer == "yes":
            self.default.voice.speak("Great! Let's finally exit to floor {}.".format(floor))
            self.default.controllers.torso_controller.sync_reach_to(0.2)
            return 'success'
        else:
            self.default.voice.speak("Does anyone want to exit on this floor?")
            if RASA:
                try:
                    new_answer = self.affirm()
                except:
                    new_answer = "yes"
            else:
                # get the new_answer
                req = AudioAndTextInteractionRequest()
                req.action = "BUTTON_PRESSED"
                req.subaction = "confirm_button"
                req.query_text = "SOUND:PLAYING:PLEASE"
                resp = self.default.speech(req)
                new_answer = resp.result
                rospy.loginfo("The response of asking the people in check available exit  new_answer is {}".format(resp.result))

            if new_answer == "yes":
                self.default.voice.speak("Great! Let's see how this will happen.")
                self.default.controllers.torso_controller.sync_reach_to(0.2)
                return 'wait'

            self.default.voice.speak("We can continue this lift journey")
            self.default.controllers.torso_controller.sync_reach_to(0.2)
            return 'failed'
