import smach
from .states import GoToFinish, StartPhase3, Encounter

class Phase3(smach.StateMachine):
    def __init__(self, default):
        smach.StateMachine.__init__(self, outcomes=['success'])

        with self:
            smach.StateMachine.add('START_PHASE_3', StartPhase3(default), transitions={'success' : 'Encounter'})
            smach.StateMachine.add('Encounter', Encounter(default), transitions={'success' : 'GO_TO_FINISH', 'failed' : 'Encounter'})
            smach.StateMachine.add('GO_TO_FINISH', GoToFinish(default), transitions={'success' : 'success'})
