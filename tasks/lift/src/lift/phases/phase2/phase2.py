import smach
from .states import GoToLift, CheckOpenDoor, NavigateInLift, AnnounceArrival, WaitForPeople, FacePerson, DeclareFloor, StartPhase2, CheckAvailableExit, Negotiate
class Phase2(smach.StateMachine):
    def __init__(self, controllers, voice, yolo):
        smach.StateMachine.__init__(self, outcomes=['success'])

        with self:
            pass
            smach.StateMachine.add('START_PHASE_2', StartPhase2(controllers, voice), transitions={'success': 'GO_TO_LIFT'})
            smach.StateMachine.add('GO_TO_LIFT', GoToLift(controllers, voice), transitions={'success': 'ANNOUNCE_ARRIVAL'})
            smach.StateMachine.add('ANNOUNCE_ARRIVAL', AnnounceArrival(controllers, voice), transitions={'success': 'CHECK_OPEN_DOOR'})
            smach.StateMachine.add('CHECK_OPEN_DOOR', CheckOpenDoor(controllers, voice), transitions={'success': 'WAIT_FOR_PEOPLE', 'failed': 'CHECK_OPEN_DOOR'})
            smach.StateMachine.add('WAIT_FOR_PEOPLE', WaitForPeople(controllers, voice, yolo), transitions={'success': 'NAVIGATE_IN_LIFT', 'failed': 'WAIT_FOR_PEOPLE'})
            smach.StateMachine.add('NAVIGATE_IN_LIFT', NavigateInLift(controllers, voice), transitions={'success': 'FACE_PERSON'})
            smach.StateMachine.add('FACE_PERSON', FacePerson(controllers, voice, yolo), transitions={'success': 'DECLARE_FLOOR', 'failed': 'WAIT_FOR_PEOPLE'})
            smach.StateMachine.add('DECLARE_FLOOR', DeclareFloor(controllers, voice), transitions={'success': 'CHECK_OPEN_DOOR', 'failed': 'DECLARE_FLOOR'})
            smach.StateMachine.add('CHECK_OPEN_DOOR', CheckOpenDoor(controllers, voice), transitions={'success': 'FACE_PERSON', 'failed': 'CHECK_OPEN_DOOR'})
            smach.StateMachine.add('FACE_PERSON', FacePerson(controllers, voice, yolo), transitions={'success': 'CHECK_AND_NEGOTIATE', 'failed': 'CHECK_OPEN_DOOR'})
            smach.StateMachine.add('CHECK_AVAILABLE_EXIT', CheckAvailableExit(controllers, voice), transitions={'success':'NEGOTIATE', 'failed': 'CHECK_OPEN_DOOR'})
            smach.StateMachine.add('NEGOTIATE', Negotiate(controllers, voice), transitions={'success':'success', 'failed': 'CHECK_OPEN_DOOR'})

