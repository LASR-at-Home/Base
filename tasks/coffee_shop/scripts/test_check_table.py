#!/usr/bin/env python3
import smach
from coffee_shop.phases.phase_1.states import CheckTable
import rospy
from tiago_controllers import HeadController
from lasr_voice import Voice

if __name__ == "__main__":
    rospy.init_node("test_check_table")
    sm = smach.StateMachine(outcomes=['end', 'not_finished'])
    with sm:
        sm.add('CHECK_TABLE', CheckTable(HeadController(), Voice(), debug=False), transitions={'finished' : 'end'})
    sm.execute()
