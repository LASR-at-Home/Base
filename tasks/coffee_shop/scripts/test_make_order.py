#!/usr/bin/env python3
import smach
from coffee_shop.phases.phase_2.states import MakeOrder, CheckOrder, InvalidateOrder
import rospy
from lasr_voice import Voice

if __name__ == "__main__":
    rospy.init_node("test_check_table")
    sm = smach.StateMachine(outcomes=['end'])
    voice_controller = Voice()
    with sm:
        sm.add('MAKE_ORDER', MakeOrder(voice_controller), transitions={'done' : 'CHECK_ORDER'})
        sm.add('CHECK_ORDER', CheckOrder(voice_controller), transitions={'correct': 'end', 'incorrect': 'INVALIDATE_ORDER'})
        smach.StateMachine.add('INVALIDATE_ORDER', InvalidateOrder(voice_controller), transitions={'done': 'CHECK_ORDER'})
    sm.execute()
    rospy.signal_shutdown("down")
