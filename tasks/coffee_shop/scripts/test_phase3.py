#!/usr/bin/env python3
import smach
from coffee_shop.phases.phase_3 import Phase3
import rospy
from lasr_voice import Voice
from tiago_controllers import BaseController

if __name__ == "__main__":
    rospy.init_node("test_wait_for_person")
    sm = smach.StateMachine(outcomes=['end'])
    with sm:
        sm.add('PHASE_3', Phase3(BaseController(), Voice()), transitions={'done' : 'end'})
    sm.execute()
