#!/usr/bin/env python3
import smach
from coffee_shop.phases.phase_2.states import TakeOrder
import rospy
from lasr_voice import Voice
from tiago_controllers import HeadController
from lasr_speech.srv import Speech

if __name__ == "__main__":
    rospy.init_node("test_take_order")
    sm = smach.StateMachine(outcomes=['end'])
    rospy.wait_for_service("/lasr_speech/transcribe_and_parse")
    speech = rospy.ServiceProxy("/lasr_speech/transcribe_and_parse", Speech)

    with sm:
        sm.add('TAKE_ORDER', TakeOrder(HeadController(), Voice(), speech), transitions={'done' : 'end'})
    sm.execute()
