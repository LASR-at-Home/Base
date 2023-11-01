#!/usr/bin/env python3
import smach
from lift.phases import Phase1, Phase2, Phase3
from lift.default import Default
from receptionist.states import Start

class Receptionist(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])
        self.default = Default()

        with self:
            smach.StateMachine.add('START', Phase1(self.default), transitions={'success' : 'success'})
  