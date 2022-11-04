#!/usr/bin/env python3

import rospy
import smach
from robocup_receptionist.states.detect_seatable_position_state import DetectSeatablePositionState
import rosservice
from std_srvs.srv import Empty
from pal_interaction_msgs.msg import TtsAction, TtsGoal, TtsText
import actionlib
from dialogflow_speech.utils import talk


class PointAtSeatableState(smach.State):

    def __init__(self, base_controller):
        smach.State.__init__(
            self,
            outcomes=['finished_pointing'],
            input_keys=["seatable_pose"]
        )
        self.base_controller = base_controller

        if "point" in rosservice.get_service_list():
            rospy.wait_for_service("point")
            self.point = rospy.ServiceProxy("point", Empty)
        else:
            self.point = lambda : None
        if rospy.get_published_topics(namespace='/move_base'):
            self.face = self.base_controller.sync_face_to
        else:
            self.face = lambda x : None

    def execute(self, userdata):

        self.face(userdata.seatable_pose.point.x, userdata.seatable_pose.point.y)
        self.point()

        return 'finished_pointing'

class SaySeatState(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded']
        )
        self.tts_client = actionlib.SimpleActionClient('/tts', TtsAction)
    
    def execute(self, userdata):
        talk("Please take a seat here, my newly acquainted friend.")

        return 'succeeded'

class SeatGuestSM(smach.StateMachine):
    
    def __init__(self, base_controller, head_controller):
        smach.StateMachine.__init__(self, outcomes=['guest_seated', 'guest_not_seated'], input_keys=[])

        with self:
            smach.StateMachine.add(
                'GET_AVAILABLE_SEAT',
                DetectSeatablePositionState(head_controller),
                transitions={'got_available_seat' : 'POINT_AT_SEATABLE_POSE', 'no_available_seat' : 'guest_not_seated'},
                remapping={'seatable_pose' : 'seatable_pose'}
            )

            smach.StateMachine.add(
                'POINT_AT_SEATABLE_POSE',
                PointAtSeatableState(base_controller),
                transitions={'finished_pointing' : 'SAY_SEAT'},
                remapping={'seatable_pose' : 'seatable_pose'}
            )

            smach.StateMachine.add(
                'SAY_SEAT',
                SaySeatState(),
                transitions={'succeeded' : 'guest_seated'}
            )
