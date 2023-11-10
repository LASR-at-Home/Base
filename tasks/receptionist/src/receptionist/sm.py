#!/usr/bin/env python3
import smach
from receptionist.states.start import Start
from receptionist.states.waitforperson import WaitForPerson
from receptionist.states.askfordrink import AskForDrink
from receptionist.default import Default
#from receptionist.states.end import End


class Receptionist(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])
        self.default = Default()

        with self:
           # smach.StateMachine.add('START', Start(self.default), transitions={'success' : 'WAIT_FOR_PERSON'})
            smach.StateMachine.add('WAIT_FOR_PERSON', WaitForPerson(self.default),transitions={'success' : 'ASK_FOR_DRINK'})
            smach.StateMachine.add('ASK_FOR_DRINK', AskForDrink(self.default),transitions={'failed':'ASK_FOR_DRINK','waitforguest2':'WAIT_FOR_PERSON','success':'success'})
            #smach.StateMachine.add('END',End(self.default),transitions={'success':'success'})
  