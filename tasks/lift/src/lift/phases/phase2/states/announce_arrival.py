#!/usr/bin/env python3
import smach, rospy

class AnnounceArrival(smach.State):
    def __init__(self, controllers, voice):
        smach.State.__init__(self, outcomes=['success'])
        self.voice = voice

    def execute(self, userdata):
        self.voice.speak("Just a quick update... I arrived at the lift. Waiting for the doors to open")
        self.voice.speak("I will fill in the time with some elevator jokes")

        return 'success'