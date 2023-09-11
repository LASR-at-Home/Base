#!/usr/bin/env python3
import smach
from coffee_shop.phases.phase_3.states import LookForPerson, GreetPerson, GoToPerson
import rospy
from coffee_shop.context import Context

if __name__ == "__main__":
    rospy.init_node("test_wait_for_person")
    sm = smach.StateMachine(outcomes=['end'])
    context = Context()

    with sm:
        sm.add('LOOK_FOR_PERSON', LookForPerson(context), transitions={'found' : 'GO_TO_PERSON', 'not found' : 'LOOK_FOR_PERSON'})
        sm.add('GO_TO_PERSON', GoToPerson(context), transitions={'done' : 'GREET_PERSON'})
        sm.add('GREET_PERSON', GreetPerson(context), transitions={'done' : 'end'})
    sm.execute()
