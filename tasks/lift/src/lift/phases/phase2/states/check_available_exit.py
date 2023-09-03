#!/usr/bin/env python3
import smach
import rospy

class CheckAvailableExit(smach.State):
    def __init__(self, controllers, voice):
        smach.State.__init__(self, outcomes=['success', 'failed', 'wait'])
        self.voice = voice

    def execute(self, userdata):
        floor = rospy.get_param("/floor/number")
        if floor == 0:
            floor = 1
        self.voice.speak("I would love to go to the floor {}.".format(floor))
        self.voice.speak(" Is that floor {}?".format(floor))

        # get the answer
        answer = "yes"
        new_answer = "no"
        if answer == "yes":
            self.voice.speak("Great! Let's finally exit to floor {}.".format(floor))
            return 'success'
        elif answer == "no":
            self.voice.speak("Some people")
        else:
            # self.voice.speak("I will wait more")
            self.voice.speak("Does anyone want to exit on this floor?")
            if new_answer == "yes":
                self.voice.speak("Great! Let's see how this will happen.")
                return 'wait'
            return 'failed'
