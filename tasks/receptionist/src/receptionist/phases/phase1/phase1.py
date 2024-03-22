import smach
from .states import Start
from .states import GoToWaitForPerson
from .states import GoToPerson
from lasr_skills import WaitForPersonInArea

class Phase1(smach.StateMachine):
    def __init__(self, default):
        smach.StateMachine.__init__(self, outcomes=['success'])

        with self:
            #smach.StateMachine.add('START', Start(default), transitions={'success': 'success'})
            smach.StateMachine.add("GO_TO_WAIT_FOR_PERSON",GoToWaitForPerson(self.default), transitions={'failed':'GO_TO_WAIT_FOR_PERSON','succeeded': 'WAIT_FOR_PERSON'})
            smach.StateMachine.add('WAIT_FOR_PERSON', WaitForPersonInArea() ,transitions={'failed' : 'WAIT_FOR_PERSON','succeeded':'GO_TO_PERSON'})
            smach.StateMachine.add('GO_TO_PERSON', GoToPerson(self.default),transitions={'succeeded':'succeeded'})


            