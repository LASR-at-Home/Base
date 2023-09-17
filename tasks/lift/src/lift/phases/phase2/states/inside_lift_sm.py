import rospy
import smach
import sys
from .check_available_exit import CheckAvailableExit
from .check_open_door import CheckOpenDoor
from .face_person import FacePerson
from .navigate_in_lift import NavigateInLift
from .negotiate import Negotiate
from .schedule_going_out import ScheduleGoingOut


class InsideLiftSM(smach.StateMachine):
    def __init__(self, default):
        smach.StateMachine.__init__(self, outcomes=['success'])

        with self:
            smach.StateMachine.add('CHECK_OPEN_DOOR', CheckOpenDoor(default), transitions={'success': 'FACE_PERSON', 'failed': 'CHECK_OPEN_DOOR'})
            smach.StateMachine.add('FACE_PERSON', FacePerson(default), transitions={'success': 'CHECK_AVAILABLE_EXIT', 'failed': 'FACE_PERSON'})
            smach.StateMachine.add('CHECK_AVAILABLE_EXIT', CheckAvailableExit(default), transitions={'success':'NEGOTIATE', 'failed': 'CHECK_OPEN_DOOR', 'wait': 'SCHEDULE_GOING_OUT'})

            smach.StateMachine.add('SCHEDULE_GOING_OUT', ScheduleGoingOut(default), transitions={'success':'NAVIGATE_IN_LIFT', 'failed': 'CHECK_OPEN_DOOR'})
            smach.StateMachine.add('NAVIGATE_IN_LIFT', NavigateInLift(default), transitions={'success': 'CHECK_OPEN_DOOR'})

            smach.StateMachine.add('NEGOTIATE', Negotiate(default), transitions={'success':'success', 'failed': 'NEGOTIATE'})


if __name__ == "__main__":
    rospy.init_node("InsideLiftSM", sys.argv)
    sm = InsideLiftSM(default=None)
    sm.execute(smach.UserData())
    rospy.spin()
