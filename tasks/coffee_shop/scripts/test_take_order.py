#!/usr/bin/env python3
import smach
from coffee_shop.phases.phase_2.states import TakeOrder
import rospy
from coffee_shop.context import Context

if __name__ == "__main__":
    rospy.init_node("test_take_order")
    sm = smach.StateMachine(outcomes=['end'])
    context = Context()
    context.target_object_remappings = {"coffee_cup" : "coffee"}

    with sm:
        sm.add('TAKE_ORDER', TakeOrder(context), transitions={'done' : 'end'})
    sm.execute()
