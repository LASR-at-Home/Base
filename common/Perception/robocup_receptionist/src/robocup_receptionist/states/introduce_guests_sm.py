#!/usr/bin/env python3

import rospy
import sys
import smach
from smach_ros import IntrospectionServer
from robocup_receptionist.states.redetect_state import RedetectState
from smach import CBState

from robocup_receptionist.states.introduce_guest_sm import IntroduceGuestSM
from robocup_receptionist.models.guest import Guest

class IntroduceGuestsSM(smach.StateMachine):
    
    def __init__(self, base_controller, head_controller, torso_controller):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'no_people'], input_keys=['guest_list'], output_keys=['guest_list'])

        with self:
            smach.StateMachine.add('REDETECT',
                                   RedetectState(head_controller, torso_controller),
                                   transitions={'people_recognized': 'INTRODUCE_GUESTS',
                                                'no_people_recognized': 'INTRODUCE_GUESTS'},
                                   remapping={'introduction_combinations' : 'introduction_combinations', 'guest_list' : 'guest_list'}
            )

            smach.StateMachine.add('INTRODUCE_GUESTS',
                                    IntroduceGuestSM(base_controller, head_controller),
                                    transitions={'succeeded' : 'succeeded'},
                                    remapping={'introduction_combinations' : 'introduction_combinations'}
            )

if __name__ == "__main__":
    rospy.init_node("receptionist_state_machine", sys.argv)
    sm = IntroduceGuestsSM()
    sis = IntrospectionServer('iserver', sm, 'SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
    rospy.loginfo('I have completed execution with outcome: ')
    rospy.loginfo(outcome)