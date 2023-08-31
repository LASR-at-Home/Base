#!/usr/bin/env python3
import smach
import rospy
from lasr_voice.voice import Voice

class GreetPerson(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['done'])
        self.context = context

    def execute(self, userdata):
        self.context.voice_controller.sync_tts("Hi there! My name is TIAGO. Please follow me, I'll guide you to a table.")
        return 'done'