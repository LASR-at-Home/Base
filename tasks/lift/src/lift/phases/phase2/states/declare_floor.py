#!/usr/bin/env python3
import smach
import rospy

class DeclareFloor(smach.State):
    def __init__(self, controllers, voice, speech):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.voice = voice
        self.speech = speech
    #Whisper listening: 
    def listen(self):
        resp = self.speech()
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
        if resp["intent"]["name"] != "affirm":
            self.voice.speak("Sorry, I didn't get that")
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




    def execute(self, userdata):
        floor = rospy.get_param("/floor/number")
        rospy.set_param("/in_lift/status", True)
        if floor == 0:
            floor = 1
        #self.voice.speak("I would love to go to the floor {}.".format(floor))
        #self.voice.speak(" Is the button for the floor {} selected?".format(floor))
        
        answer = self.affirm()

        print("Answer from Speech: ", answer)
        #rasa get answer: 

        # From answer: 
        answer = "yes"
        if answer == "yes":
            self.voice.speak("Great!")
            return 'success'
        else:
            self.voice.speak("I will wait more")
            return 'failed'
