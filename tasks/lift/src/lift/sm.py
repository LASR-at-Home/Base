#!/usr/bin/env python3
import smach
from lift.phases import Phase1
from tiago_controllers.controllers import Controllers
from lasr_voice.voice import Voice

class Lift(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])

        self.controllers = Controllers()
        # self.voice = Voice()
        self.voice = "hello"

        with self:
            smach.StateMachine.add('PHASE1', Phase1(self.controllers, self.voice), transitions={'success' : 'success'})
            # smach.StateMachine.add('PHASE1', Phase1(self.controllers, self.voice), transitions={'done' : 'PHASE2'})
            # smach.StateMachine.add('PHASE2', Phase2(self.controllers, self.voice), transitions={'done' : 'success'})