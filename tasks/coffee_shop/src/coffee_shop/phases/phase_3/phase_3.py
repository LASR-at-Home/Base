import smach
from .states import GoToWaitLocation, WaitForPerson, GreetPerson, GuidePerson

class Phase3(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=[])

        with self:
            
            smach.StateMachine.add('GO_TO_WAIT_LOCATION', GoToWaitLocation(), transitions={'succeeded' : 'WAIT_FOR_PERSON'})
            
            smach.StateMachine.add('WAIT_FOR_PERSON', WaitForPerson(), transitions={'succeeded', 'GREET_PERSON'})

            smach.StateMachine.add('GREET_PERSON', GreetPerson(), transitions={'succeeded' : 'GUIDE_PERSON'})

            smach.StateMachine.add('GUIDE_PERSON', GuidePerson(), transitions={'succeeded' : 'end'})
