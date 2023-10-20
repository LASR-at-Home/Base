#!/usr/bin/env python3
import rospy
import smach
from lasr_navigate_to_known_person.states import GoTo, Scan
from tiago_controllers.controllers import ReachToRadius, HeadController


class ScanAndGoTo(smach.StateMachine):
    def __init__(self, base_controller: ReachToRadius, head_controller: HeadController):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        self.base_controller = base_controller
        self.head_controller = head_controller
        self.userdata.prev = 'Scan'

        with self:
            smach.StateMachine.add('SCAN_ROOM', Scan(self.head_controller),
                                   transitions={
                                       'succeeded': 'GO_TO_HOST',
                                       'failed': 'failed'
                                   },
                                   remapping={
                                       'location': 'location',
                                       'prev': 'prev'
                                   })

            smach.StateMachine.add('GO_TO_HOST', GoTo(self.base_controller),
                                   transitions={
                                       'succeeded': 'succeeded',
                                       'failed': 'failed'
                                   },
                                   remapping={
                                       'location': 'location',
                                       'prev': 'prev'
                                   })


if __name__ == '__main__':
    rospy.init_node("navigate_to_person_node", anonymous=True)
    base_controller = ReachToRadius()
    head_controller = HeadController()
    sm = smach.StateMachine(outcomes=['success', 'no_people'])

    with sm:
        smach.StateMachine.add('SCAN_AND_GO_TO',
                               ScanAndGoTo(base_controller, head_controller),
                               transitions={'succeeded': 'success',
                                            'failed': 'SCAN_AND_GO_TO'})

    outcome = sm.execute()
    rospy.loginfo('I have completed execution with outcome: ')
    rospy.loginfo(outcome)
