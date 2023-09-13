#!/usr/bin/env python3
import smach
from coffee_shop.phases.phase_2.states import TakeOrder
import rospy
from coffee_shop.context import Context
import sys

if __name__ == "__main__":
    rospy.init_node("test_take_order")
    sm = smach.StateMachine(outcomes=['end'])
    context = Context(sys.argv[1])
    context.target_object_remappings = {"coffee_cup" : "coffee", "pringles" : "pringles", "small_pringles" : "small pringles"}
    context.current_table = "table0"
    context.tables[context.current_table]["people"] = [[0.114, 2.16, 1.8]]
    with sm:
        sm.add('TAKE_ORDER', TakeOrder(context), transitions={'done' : 'end'})
    sm.execute()
