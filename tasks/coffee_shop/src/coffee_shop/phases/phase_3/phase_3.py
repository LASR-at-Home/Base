import smach
from .states import GoToWaitLocation, WaitForPerson, GreetPerson, GuidePerson

class Phase3(smach.StateMachine):
    def __init__(self, base_controller):
        smach.StateMachine.__init__(self, outcomes=['done'])

        with self:
            
            smach.StateMachine.add('GO_TO_WAIT_LOCATION', GoToWaitLocation(base_controller), transitions={'done' : 'WAIT_FOR_PERSON'})
            
            smach.StateMachine.add('WAIT_FOR_PERSON', WaitForPerson(), transitions={'done' : 'GREET_PERSON'})

            smach.StateMachine.add('GREET_PERSON', GreetPerson(), transitions={'done' : 'GUIDE_PERSON'})

            smach.StateMachine.add('GUIDE_PERSON', GuidePerson(base_controller), transitions={'done' : 'done'})
