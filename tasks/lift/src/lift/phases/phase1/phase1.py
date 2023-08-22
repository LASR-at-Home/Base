import smach
from .states import SpeakWithGroup, GoToLift, CheckOpenDoor, NavigateInLift

class Phase1(smach.StateMachine):
    def __init__(self, controllers, voice):
        smach.StateMachine.__init__(self, outcomes=['success'])

        with self:
            pass
            # smach.StateMachine.add('SPEAK_WITH_GROUP', SpeakWithGroup(controllers, voice), transitions={'success' : 'GO_TO_LIFT', 'failed' : 'SPEAK_WITH_GROUP'})
            # smach.StateMachine.add('GO_TO_LIFT', GoToLift(controllers, voice), transitions={'success' : 'CHECK_OPEN_DOOR'})
            # smach.StateMachine.add('CHECK_OPEN_DOOR', CheckOpenDoor(controllers, voice), transitions={'success' : 'NAVIGATE_IN_LIFT', 'failed' : 'CHECK_OPEN_DOOR'})
            smach.StateMachine.add('NAVIGATE_IN_LIFT', NavigateInLift(controllers, voice), transitions={'success' : 'success'})
