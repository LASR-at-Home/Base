import rospy, smach, sys
from .check_open_door import CheckOpenDoor
from .face_person import FacePerson
from .check_available_exit import CheckAvailableExit
from .negotiate import Negotiate
from .schedule_going_out import ScheduleGoingOut
from .navigate_in_lift import NavigateInLift


class InsideLiftSM(smach.StateMachine):
    def __init__(self, controllers, voice, yolo, cmd, speech):
        smach.StateMachine.__init__(self, outcomes=['success'])

        with self:
            smach.StateMachine.add('CHECK_OPEN_DOOR', CheckOpenDoor(controllers, voice), transitions={'success': 'FACE_PERSON', 'failed': 'CHECK_OPEN_DOOR'})
            smach.StateMachine.add('FACE_PERSON', FacePerson(controllers, voice, yolo, cmd), transitions={'success': 'CHECK_AVAILABLE_EXIT', 'failed': 'FACE_PERSON'})
            smach.StateMachine.add('CHECK_AVAILABLE_EXIT', CheckAvailableExit(controllers, voice, speech), transitions={'success':'NEGOTIATE', 'failed': 'CHECK_OPEN_DOOR', 'wait': 'SCHEDULE_GOING_OUT'})

            smach.StateMachine.add('SCHEDULE_GOING_OUT', ScheduleGoingOut(controllers, voice, speech), transitions={'success':'NAVIGATE_IN_LIFT', 'failed': 'CHECK_OPEN_DOOR'})
            smach.StateMachine.add('NAVIGATE_IN_LIFT', NavigateInLift(controllers, voice), transitions={'success': 'CHECK_OPEN_DOOR'})

            smach.StateMachine.add('NEGOTIATE', Negotiate(controllers, voice), transitions={'success':'success', 'failed': 'NEGOTIATE'})



if __name__ == "__main__":
    rospy.init_node("InsideLiftSM", sys.argv)
    sm = InsideLiftSM(controllers=None, voice=None, yolo=None)
    sm.execute(smach.UserData())
    rospy.spin()
