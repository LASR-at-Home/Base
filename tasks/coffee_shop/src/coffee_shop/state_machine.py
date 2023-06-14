#!/usr/bin/env python3
import smach
from coffee_shop.phases import Phase1, Phase2, Phase3

class CoffeeShop(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])
        with self:
            smach.StateMachine.add('PHASE_1', Phase1(), transitions={'succeeded' : 'PHASE_3'})
            smach.StateMachine.add('PHASE_3', Phase3(), transitions={'succeeded' : 'end'})
