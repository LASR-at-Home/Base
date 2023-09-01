#!/usr/bin/env python3
import smach
import rospy

class CheckAvailableExit(smach.State):
    def __init__(self, controllers, voice):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.voice = voice

    def execute(self, userdata):
        floor = rospy.get_param("/floor/number")
        if floor == 0:
            floor = 1
        self.voice.speak("I would love to go to the floor {}.".format(floor))
        self.voice.speak(" Is that floor {}?".format(floor))

        # get the answer
        answer = "yes"
        if answer == "yes":
            self.voice.speak("Great! Let's finally exit to floor {}.".format(floor))
            return 'success'
        else:
            self.voice.speak("I will wait more")
            return 'failed'
