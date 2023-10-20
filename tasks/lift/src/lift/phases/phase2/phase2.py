import smach
from .states import GoToLift, CheckOpenDoor, NavigateInLift, AnnounceArrival, WaitForPeople, FacePerson, DeclareFloor, StartPhase2, CheckAvailableExit, Negotiate, InsideLiftSM
class Phase2(smach.StateMachine):
    def __init__(self, default):
        smach.StateMachine.__init__(self, outcomes=['success'])

        with self:
            smach.StateMachine.add('START_PHASE_2', StartPhase2(default), transitions={'success': 'GO_TO_LIFT'})
            smach.StateMachine.add('GO_TO_LIFT', GoToLift(default), transitions={'success': 'ANNOUNCE_ARRIVAL'})
            smach.StateMachine.add('ANNOUNCE_ARRIVAL', AnnounceArrival(default), transitions={'success': 'CHECK_OPEN_DOOR'})
            smach.StateMachine.add('CHECK_OPEN_DOOR', CheckOpenDoor(default), transitions={'success': 'WAIT_FOR_PEOPLE', 'failed': 'CHECK_OPEN_DOOR'})
            smach.StateMachine.add('WAIT_FOR_PEOPLE', WaitForPeople(default), transitions={'success': 'NAVIGATE_IN_LIFT', 'failed': 'WAIT_FOR_PEOPLE'})
            smach.StateMachine.add('NAVIGATE_IN_LIFT', NavigateInLift(default), transitions={'success': 'FACE_PERSON'})
            smach.StateMachine.add('FACE_PERSON', FacePerson(default), transitions={'success': 'DECLARE_FLOOR', 'failed': 'FACE_PERSON'})
            smach.StateMachine.add('DECLARE_FLOOR', DeclareFloor(default), transitions={'success': 'INSIDE_LIFT_SM', 'failed': 'DECLARE_FLOOR'})

            smach.StateMachine.add('INSIDE_LIFT_SM', InsideLiftSM(default), transitions={'success': 'success'})


