#!/usr/bin/env python3
import smach
import rospy
from lasr_voice.voice import Voice

class GreetPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.voice = Voice()
    def execute(self, userdata):
        rospy.sleep(5.0)
        self.voice.sync_tts("Hi there! My name is TIAGO. Please follow me, I'll guide you to a table.")
        return 'done'

