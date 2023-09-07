import smach
from .states import SpeakWithGroup, StartPhase1

class Phase1(smach.StateMachine):
    def __init__(self, controllers, voice, cmd, pm):
        smach.StateMachine.__init__(self, outcomes=['success'])

        with self:
            pass
            smach.StateMachine.add('START_PHASE_1', StartPhase1(controllers, voice, pm), transitions={'success': 'success'})
            # smach.StateMachine.add('START_PHASE_1', StartPhase1(controllers, voice), transitions={'success' : 'SPEAK_WITH_GROUP'})
            # smach.StateMachine.add('SPEAK_WITH_GROUP', SpeakWithGroup(controllers, voice, cmd), transitions={'success' : 'success', 'failed' : 'SPEAK_WITH_GROUP'})
