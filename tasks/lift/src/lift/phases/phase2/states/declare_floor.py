#!/usr/bin/env python3
import smach
import rospy

class DeclareFloor(smach.State):
    def __init__(self, controllers, voice):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.voice = voice

    def execute(self, userdata):
        floor = rospy.get_param("/floor/number")
        rospy.set_param("/in_lift/status", True)
        if floor == 0:
            floor = 1
        self.voice.speak("I would love to go to the floor {}.".format(floor))
        self.voice.speak(" Is the button for the floor {} selected?".format(floor))

        # get the answer
        answer = "yes"
        if answer == "yes":
            self.voice.speak("Great!")
            return 'success'
        else:
            self.voice.speak("I will wait more")
            return 'failed'
