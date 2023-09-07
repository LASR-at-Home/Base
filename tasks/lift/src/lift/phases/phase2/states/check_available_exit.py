#!/usr/bin/env python3
import smach
import rospy
from interaction_module.srv import AudioAndTextInteraction, AudioAndTextInteractionRequest, \
    AudioAndTextInteractionResponse

class CheckAvailableExit(smach.State):
    def __init__(self, controllers, voice, speech):
        smach.State.__init__(self, outcomes=['success', 'failed', 'wait'])
        self.voice = voice
        self.controllers = controllers
        self.speech = speech

    def execute(self, userdata):
        floor = rospy.get_param("/floor/number")
        if floor == 0:
            floor = 1

        # ensure the head is straight
        self.controllers.head_controller.look_straight()

        self.voice.speak("I really really want to go to the floor {}.".format(floor))
        self.voice.speak(" Is that floor {}?".format(floor))

        # get the answer
        req = AudioAndTextInteractionRequest()
        req.action = "BUTTON_PRESSED"
        req.subaction = "confirm_button"
        req.query_text = "SOUND:PLAYING:PLEASE"
        resp = self.speech(req)
        rospy.loginfo("The response of asking the people in check available exit is {}".format(resp.result))
        answer = resp.result

        if answer == "yes":
            self.voice.speak("Great! Let's finally exit to floor {}.".format(floor))
            return 'success'
        else:
            self.voice.speak("Does anyone want to exit on this floor?")
            # get the new_answer
            req = AudioAndTextInteractionRequest()
            req.action = "BUTTON_PRESSED"
            req.subaction = "confirm_button"
            req.query_text = "SOUND:PLAYING:PLEASE"
            resp = self.speech(req)
            new_answer = resp.result
            rospy.loginfo("The response of asking the people in check available exit  new_answer is {}".format(resp.result))
            if new_answer == "yes":
                self.voice.speak("Great! Let's see how this will happen.")
                return 'wait'

            self.voice.speak("We can continue this lift journey")
            return 'failed'
