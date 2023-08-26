#!/usr/bin/env python3
import smach
from coffee_shop.phases import Phase1, Phase2, Phase3

class CoffeeShop(smach.StateMachine):
    def __init__(self, base_controller, head_controller, voice_controller, yolo, tf, pm, speech, shapely):
        smach.StateMachine.__init__(self, outcomes=['end'])
        with self:
            smach.StateMachine.add('PHASE_1', Phase1(base_controller, head_controller, voice_controller, yolo, tf, pm, shapely), transitions={'done' : 'PHASE_2'})
            smach.StateMachine.add('PHASE_2', Phase2(base_controller, head_controller, voice_controller, yolo, tf, pm, speech, shapely), transitions={'done' : 'PHASE_3'})
            smach.StateMachine.add('PHASE_3', Phase3(base_controller, voice_controller, yolo, tf, pm, speech, shapely), transitions={'done' : 'end'})
