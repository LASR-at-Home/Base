#!/usr/bin/env python3
import smach
import rospy
from interaction_module.srv import AudioAndTextInteraction, AudioAndTextInteractionRequest, \
    AudioAndTextInteractionResponse

class DeclareFloor(smach.State):
    def __init__(self, controllers, voice, speech):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.voice = voice
        self.speech = speech

    def execute(self, userdata):
        floor = rospy.get_param("/floor/number")
        rospy.set_param("/in_lift/status", True)
        if floor == 0:
            floor = 1

        # maybe add the counter here as well
        self.voice.speak("I would love to go to the floor {}.".format(floor))
        self.voice.speak("Can you help me by pressing the button?")
        self.voice.speak("Please answer yes or no.")
        rospy.sleep(1)

        req = AudioAndTextInteractionRequest()
        req.action = "BUTTON_PRESSED"
        req.subaction = "confirm_button"
        req.query_text = "SOUND:PLAYING:PLEASE"
        resp = self.speech(req)

        # get the answer
        if resp.result == 'yes':
            self.voice.speak("Great! Thank you for pressing the button!")
            return 'success'
        else:
            self.voice.speak("I will wait more")
            rospy.sleep(1)
            return 'failed'
