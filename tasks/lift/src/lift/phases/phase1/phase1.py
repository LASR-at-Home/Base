import smach
from .states import SpeakWithGroup, GoToLift

class Phase1(smach.StateMachine):
    def __init__(self, controllers, voice):
        smach.StateMachine.__init__(self, outcomes=['success'])

        with self:
            pass
            smach.StateMachine.add('SPEAK_WITH_GROUP', SpeakWithGroup(controllers, voice), transitions={'success' : 'GO_TO_LIFT', 'failed' : 'SPEAK_WITH_GROUP'})
            smach.StateMachine.add('GO_TO_LIFT', GoToLift(controllers, voice), transitions={'success' : 'success'})