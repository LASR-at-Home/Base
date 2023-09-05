#!/usr/bin/env python3
import smach
import rospy

class StartPhase2(smach.State):
    def __init__(self, controllers, voice):
        smach.State.__init__(self, outcomes=['success'])
        self.voice = voice

    def execute(self, userdata):
        self.voice.speak("Just a quick update... I am starting Phase 2. I am going to the lift.")

        return 'success'