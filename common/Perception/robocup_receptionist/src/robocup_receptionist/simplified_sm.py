#!/usr/bin/env python3


import rospy
import sys
import smach
from smach_ros import IntrospectionServer
from robocup_receptionist.models import Guest

from robocup_receptionist.states import GoToStart, WaitForGuestState, \
                                        IntroduceGuestsSM, CollectInfoAndScanSM, ScanAndGoToHostSM, SeatGuestSM, FaceVisibleState
from tiago_controllers import BaseController, HeadController, TorsoController
from smach import CBState

from dialogflow_speech.utils import talk

TOTAL_GUESTS = 2

class ReceptionistSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted'])
        self.current_guest = 0
        self.total_guests = TOTAL_GUESTS
        host_name = rospy.get_param("host_name", "host")
        host_drink = rospy.get_param("host_drink", "ice tea")
        self.userdata.guest_list = [Guest(host_name, host_drink)]
        use_navigation = False

        # Create only one instance of the controllers and pass by reference
        # Only enable navigation if 'move_base' is a published topic.
        if rospy.get_published_topics(namespace='/move_base'):
            print("using navigation")
            base_controller = BaseController()
            head_controller = HeadController()
            torso_controller = TorsoController()
            use_navigation = True
        else:
            base_controller = None
            head_controller = None
            torso_controller = None
        with self:

            if use_navigation:
                # ! Go to the start position, and wait.
                smach.StateMachine.add('GO_TO_ENTRANCE',
                                        GoToStart(base_controller, head_controller, torso_controller),
                                        transitions = {'succeeded' : 'WAIT_FOR_GUEST'}
                )
            smach.StateMachine.add('WAIT_FOR_GUEST', WaitForGuestState(base_controller),
                                    transitions = {'human_in_range' : 'CHECK_FACE_VISIBILITY', 'not_human_in_range' : 'WAIT_FOR_GUEST'},
                                    remapping = {'person_bb' : 'person_bb'}
            )

            smach.StateMachine.add('CHECK_FACE_VISIBILITY', 
                                    FaceVisibleState(torso_controller),
                                    transitions={
                                        'redetection_required' : 'WAIT_FOR_GUEST',
                                        'no_redetection_required' : 'COLLECT_AND_SCAN'
                                    },
                                    remapping = {
                                        'person_bb' : 'person_bb'
                                    }
            )

            # ! Collect person attributes.
            smach.StateMachine.add('COLLECT_AND_SCAN',
                                   CollectInfoAndScanSM(base_controller),
                                   transitions={ 'finished_collection_and_scan' : 'ADD_GUEST_TO_LIST', 'wait_for_guest' : 'WAIT_FOR_GUEST'},
                                   remapping={'current_guest':'current_guest'}
                                   )

            

            @smach.cb_interface(outcomes=['succeeded'], input_keys=['current_guest', 'guest_list'], output_keys=['current_guest', 'guest_list'])
            def add_guest_to_list(userdata):
                print(userdata.guest_list)
                userdata.guest_list.append(userdata.current_guest)
                print(userdata.guest_list)
                talk("Now, please follow me. I will guide you to the host.")
                return 'succeeded'

            smach.StateMachine.add('ADD_GUEST_TO_LIST', CBState(add_guest_to_list),
                                   transitions={'succeeded': 
                                    'SEAT_PERSON' if not use_navigation else 'SCAN_AND_GO_TO_HOST'},
                                   remapping={'current_guest': 'current_guest',
                                              'guest_list':'guest_list'}
            )

            if use_navigation:
                # ! Locate and go to host.
                smach.StateMachine.add('SCAN_AND_GO_TO_HOST',
                                        ScanAndGoToHostSM(base_controller, head_controller).sm,
                                        transitions={
                                                        'succeeded': 'SEAT_PERSON',
                                                        'failed': 'aborted'
                                                    }
                )


            # # ! Seat person
            smach.StateMachine.add('SEAT_PERSON',
                                    SeatGuestSM(base_controller, head_controller),
                                    transitions={
                                                    'guest_seated': 'INTRODUCE_PERSONS',
                                                    'guest_not_seated' : 'INTRODUCE_PERSONS'
                                                },
                                   remapping={'current_guest': 'current_guest',
                                              'guest_list':'guest_list'}
            )

            #! introduce person
            smach.StateMachine.add('INTRODUCE_PERSONS',
                                    IntroduceGuestsSM(base_controller, head_controller, torso_controller),
                                    transitions={'succeeded': 'CHECK_TERMINATION', 'no_people': 'aborted'},
                                    remapping={'guest_list': 'guest_list'}
            )

            # Check for task convergence
            @smach.cb_interface(outcomes=['continue', 'finished'])
            def check_termination(ud):
                self.current_guest+=1
                return 'finished' if self.current_guest >= self.total_guests else 'continue'
            smach.StateMachine.add('CHECK_TERMINATION', CBState(check_termination),
                                   transitions={
                                       'finished': 'succeeded',
                                       'continue':'GO_TO_ENTRANCE' if use_navigation else 'COLLECT_AND_SCAN'})

if __name__ == "__main__":
    rospy.init_node("receptionist_state_machine", sys.argv)
    sm = ReceptionistSM()
    sis = IntrospectionServer('iserver', sm, 'SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
    rospy.loginfo('I have completed execution with outcome: ')
    rospy.loginfo(outcome)
