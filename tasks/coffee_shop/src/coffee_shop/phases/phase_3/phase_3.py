import smach
from .states import GoToWaitLocation, LookForPerson, GoToPerson, GreetPerson, GuidePerson

class Phase3(smach.StateMachine):
    def __init__(self, base_controller, voice_controller, yolo, tf, pm):
        smach.StateMachine.__init__(self, outcomes=['done'])

        with self:
            
            smach.StateMachine.add('GO_TO_WAIT_LOCATION', GoToWaitLocation(base_controller), transitions={'done' : 'done', 'not done' : 'LOOK_FOR_PERSON'})
            
            smach.StateMachine.add('LOOK_FOR_PERSON', LookForPerson(yolo, tf), transitions={'found' : 'GO_TO_PERSON', 'not found' : 'LOOK_FOR_PERSON'})

            smach.StateMachine.add('GO_TO_PERSON', GoToPerson(base_controller), transitions={'done' : 'GREET_PERSON'})

            smach.StateMachine.add('GREET_PERSON', GreetPerson(voice_controller), transitions={'done' : 'GUIDE_PERSON'})

            smach.StateMachine.add('GUIDE_PERSON', GuidePerson(base_controller, voice_controller), transitions={'done' : 'GO_TO_WAIT_LOCATION'})
