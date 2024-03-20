#!/usr/bin/env python3
import smach
from receptionist.states import Start
from receptionist.states import AskForName
from receptionist.states import AskForDrink
from receptionist.states import End
from receptionist import Default
from receptionist.states import GoToPerson
from receptionist.states import GoToSeatingArea
from receptionist.states import LookForSeats
from receptionist.states import GoToWaitForPerson
from lasr_skills import WaitForPersonInArea
#from receptionist.states.end import End


class Receptionist(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])
        self.default = Default()
        self.userdata.area_polygon = [[1.94, 0.15], [2.98, 0.28], [3.08, -0.68], [2.06, -0.84]]
        self.userdata.depth_topic = "/xtion/depth_registered/points"
        with self:
            smach.StateMachine.add('START', Start(self.default), transitions={'succeeded' : 'GO_TO_WAIT_FOR_PERSON'})
            smach.StateMachine.add("GO_TO_WAIT_FOR_PERSON",GoToWaitForPerson(self.default), transitions={'succeeded': 'WAIT_FOR_PERSON'})
            smach.StateMachine.add('WAIT_FOR_PERSON', WaitForPersonInArea() ,transitions={'succeeded' : 'GO_TO_PERSON', 'failed' : 'failed'})
            smach.StateMachine.add('GO_TO_PERSON', GoToPerson(self.default),transitions={'succeeded':'ASK_FOR_NAME'})
            smach.StateMachine.add('ASK_FOR_NAME', AskForName(self.default),transitions={'failed':'ASK_FOR_NAME','succeeded':'ASK_FOR_DRINK'})
            smach.StateMachine.add('ASK_FOR_DRINK', AskForDrink(self.default),transitions={'failed':'ASK_FOR_DRINK','succeeded':'GO_TO_SEATING_AREA'})
            smach.StateMachine.add('GO_TO_SEATING_AREA', GoToSeatingArea(self.default), transitions={'succeeded' : 'LOOK_FOR_SEATS'})
            smach.StateMachine.add('LOOK_FOR_SEATS', LookForSeats(self.default), transitions={'succeeded' : 'GO_TO_WAIT_FOR_PERSON'})
            smach.StateMachine.add('END', End(self.default),transitions={'succeeded':'succeeded'})
