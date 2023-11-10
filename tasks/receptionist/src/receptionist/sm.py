#!/usr/bin/env python3
import smach
from receptionist.states.start import Start
from receptionist.states.askfordrink import AskForDrink
from receptionist.default import Default
from lasr_skills import WaitForPersonInArea
#from receptionist.states.end import End


class Receptionist(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success', 'failed'])
        self.default = Default()
        self.userdata.area_polygon = [[1.94, 0.15], [2.98, 0.28], [3.08, -0.68], [2.06, -0.84]]
        with self:
           # smach.StateMachine.add('START', Start(self.default), transitions={'success' : 'WAIT_FOR_PERSON'})
            smach.StateMachine.add('WAIT_FOR_PERSON', WaitForPersonInArea() ,transitions={'succeeded' : 'ASK_FOR_DRINK', 'failed' : 'failed'})
            smach.StateMachine.add('ASK_FOR_DRINK', AskForDrink(self.default),transitions={'failed':'ASK_FOR_DRINK','waitforguest2':'WAIT_FOR_PERSON','success':'success'})
            #smach.StateMachine.add('END',End(self.default),transitions={'success':'success'})