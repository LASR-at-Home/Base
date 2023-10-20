#!/usr/bin/env python3
import smach, rospy

class AnnounceArrival(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success'])
        self.default = default

    def execute(self, userdata):
        self.default.voice.speak("I arrived at the lift.")

        return 'success'