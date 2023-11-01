#!/usr/bin/env python3
import smach
from receptionist.states import Start, WaitForPerson, AskForDrink
from receptionist.default import Default


class Receptionist(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])
        self.default = Default()

        with self:
            smach.StateMachine.add('START', Start(self.default), transitions={'success' : 'WAIT_FOR_PERSON'})
            smach.StateMachine.add('WAIT_FOR_PERSON', WaitForPerson(self.default),transitions={'success':'ASK_FOR_DRINK'})
            smach.StateMachine.add('ASK_FOR_DRINK',AskForDrink(self.default),transitions={'success':'success'})
  