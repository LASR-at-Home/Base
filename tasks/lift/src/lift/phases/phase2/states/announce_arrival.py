#!/usr/bin/env python3
import smach, rospy

class AnnounceArrival(smach.State):
    def __init__(self, controllers, voice):
        smach.State.__init__(self, outcomes=['success'])
        self.voice = voice

    def execute(self, userdata):
        self.voice.speak("For your information - I arrived at the lift. Waiting for the doors to open")
        return 'success'