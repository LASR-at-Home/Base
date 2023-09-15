#!/usr/bin/env python3
import smach

class GoToPerson(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['done'])
        self.context = context

    def execute(self, userdata):
        self.context.voice_controller.async_tts("I am going to approach the customer")
        pose = self.context.new_customer_pose
        self.context.base_controller.sync_to_radius(pose[0], pose[1], radius = 1.5)
        return 'done'