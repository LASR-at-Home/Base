#!/usr/bin/env python3
import smach
from lift.phases import Phase1, Phase2, Phase3
from lift.default import Default

class Lift(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])
        self.default = Default()


        with self:
            # smach.StateMachine.add('PHASE1', Phase1(self.controllers, self.voice, self.cmd, self.pm), transitions={'success' : 'PHASE2'})
            smach.StateMachine.add('PHASE2', Phase2(self.default), transitions={'success' : 'PHASE3'})
            smach.StateMachine.add('PHASE3', Phase3(self.default), transitions={'success' : 'success'})