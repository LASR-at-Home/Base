import smach
from .states import GoToFinish, StartPhase3

class Phase3(smach.StateMachine):
    def __init__(self, controllers, voice):
        smach.StateMachine.__init__(self, outcomes=['success'])

        with self:
            pass
            smach.StateMachine.add('START_PHASE_3', StartPhase3(controllers, voice), transitions={'success' : 'GO_TO_FINISH'})
            smach.StateMachine.add('GO_TO_FINISH', GoToFinish(controllers, voice), transitions={'success' : 'success'})
