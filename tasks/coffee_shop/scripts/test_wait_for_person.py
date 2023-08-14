#!/usr/bin/env python3
import smach
from coffee_shop.phases.phase_3.states import LookForPerson, GreetPerson, GoToPerson
import rospy
from lasr_voice import Voice
from tiago_controllers import BaseController

if __name__ == "__main__":
    rospy.init_node("test_wait_for_person")
    sm = smach.StateMachine(outcomes=['end'])
    with sm:
        sm.add('LOOK_FOR_PERSON', LookForPerson(), transitions={'found' : 'GO_TO_PERSON', 'not found' : 'LOOK_FOR_PERSON'})
        sm.add('GO_TO_PERSON', GoToPerson(BaseController()), transitions={'done' : 'GREET_PERSON'})
        sm.add('GREET_PERSON', GreetPerson(Voice()), transitions={'done' : 'end'})
    sm.execute()
