#!/usr/bin/env python3
import smach
from coffee_shop.phases.phase_3.states import WaitForPerson, GreetPerson
import rospy
from lasr_voice import Voice

if __name__ == "__main__":
    rospy.init_node("test_wait_for_person")
    sm = smach.StateMachine(outcomes=['end'])
    with sm:
        sm.add('WAIT_FOR_PERSON', WaitForPerson(), transitions={'done' : 'GREET_PERSON'})
        sm.add('GREET_PERSON', GreetPerson(Voice()), transitions={'done' : 'end'})
    sm.execute()
