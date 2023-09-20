import smach
from .states import SpeakWithGroup, StartPhase1

class Phase1(smach.StateMachine):
    def __init__(self, default):
        smach.StateMachine.__init__(self, outcomes=['success'])

        with self:
            pass
            smach.StateMachine.add('START_PHASE_1', StartPhase1(default), transitions={'success': 'success'})