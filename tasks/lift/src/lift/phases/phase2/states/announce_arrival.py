#!/usr/bin/env python3
import smach, rospy

class AnnounceArrival(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success'])
        self.default = default

    def execute(self, userdata):
        self.default.voice.speak("For your information - I arrived at the lift. Waiting for the doors to open")
        return 'success'