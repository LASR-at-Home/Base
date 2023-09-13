#!/usr/bin/env python3
import smach

class GoCloserToPerson(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['done'])
        self.context = context

    def execute(self, userdata):
        self.context.voice_controller.sync_tts("I think there is a customer waiting. I will go and investigate.")
        pose = self.context.new_customer_pose
        self.context.base_controller.sync_to_radius(pose[0], pose[1], radius = 2)
        self.context.base_controller.sync_face_to(pose[0], pose[1])
        return 'done'