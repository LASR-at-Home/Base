#!/usr/bin/env python3
import smach
import rospy
import json

class CheckAvailableExit(smach.State):
    def __init__(self, controllers, voice,speech):
        smach.State.__init__(self, outcomes=['success', 'failed', 'wait'])
        self.voice = voice
        self.speech = speech


    def listen(self):
        resp = self.speech()
        print("Resp success: ", resp.success)
        if not resp.success:
            self.voice.speak("Sorry, I didn't get that")
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
            self.voice.speak("Sorry, I didn't get that, please say yes or no")
            return self.affirm()
        choices = resp["entities"].get("choice", None)
        if choices is None:
            self.voice.speak("Sorry, I didn't get that")
            return self.affirm()
        choice = choices[0]["value"].lower()
        if choice not in ["yes", "no"]:
            self.voice.speak("Sorry, I didn't get that")
            return self.affirm()
        return choice 

    def get_floor(self):
        resp = self.listen()
        if resp["intent"]["name"] != "check_floor_number":
            self.voice.speak("Sorry, I misheard you")
            return self.get_floor()
        floor = resp["entities"].get("floor",[])
        if not floor: 
            self.voice.speak("Sorry, I can't figure out that we're talking about floors")
            return self.get_floor()
        floor_number = int(floor[0]["value"])        
        self.voice.speak("I heard that we are on floor {}".format(floor_number))
        return floor_number

    def execute(self, userdata):
        floor = rospy.get_param("/floor/number")
        if floor == 0:
            floor = 1
        self.voice.speak("I would love to go to the floor {}.".format(floor))
        #self.voice.speak(" Is that floor {}?".format(floor))
        self.voice.speak("What floor is this?")


        try:
            floor_number = self.get_floor()
        except: 
            floor = 1000

        
        answer = "no"
        if (floor_number==floor):
            answer = "yes" 

        

        # get the answer
        #answer = "yes"
        new_answer = "no"
        if answer == "yes":
            self.voice.speak("Great! Let's finally exit to floor {}.".format(floor))
            return 'success'
        else:
            # self.voice.speak("I will wait more")
            self.voice.speak("Does anyone want to exit on this floor?")
            try: 
                new_answer = self.affirm()
            except: 
                new_answer = "yes"
            if new_answer == "yes":
                self.voice.speak("Great! Let's see how this will happen.")
                return 'wait'
            return 'failed'
