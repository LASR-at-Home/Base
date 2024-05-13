#!/usr/bin/env python3
import smach
from coffee_shop.phases.phase_3 import Phase3
import rospy
from coffee_shop.context import Context
import sys

if __name__ == "__main__":
    rospy.init_node("test_wait_for_person")
    sm = smach.StateMachine(outcomes=['end'])
    context = Context(sys.argv[1],True)
    context.current_table = "table0"
    context.tables[context.current_table]["status"] = "ready"

    with sm:
        sm.add('PHASE_3', Phase3(context), transitions={'done' : 'end'})
    sm.execute()
