#!/usr/bin/env python3
import smach
from lift.phases import Phase1, Phase2, Phase3
from lift.default import Default
from std_msgs.msg import Empty, Int16

class Lift(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])
        self.default = Default()

        # self.default.datahub_ping.publish(Empty())
        # self.default.datahub_start_episode.publish(Empty())


        with self:
            smach.StateMachine.add('PHASE1', Phase1(self.default), transitions={'success' : 'PHASE2'})
            smach.StateMachine.add('PHASE2', Phase2(self.default), transitions={'success' : 'PHASE3'})
            smach.StateMachine.add('PHASE3', Phase3(self.default), transitions={'success' : 'success'})