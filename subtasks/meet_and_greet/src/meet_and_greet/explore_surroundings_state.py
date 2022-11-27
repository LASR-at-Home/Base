#!/usr/bin/env python3
# coding=utf-8
import smach
import rospy
import random
from tiago_controllers.helpers import get_pose_from_param, is_running
from tiago_controllers.controllers import Controllers

class ExploreSurroundingsState(smach.State):
    '''
        Wonders around and detects people
    '''

    def __init__(self, controllers):
        smach.State.__init__(self,
                             outcomes=['finished_exploring']
                             )

        self.controllers = controllers
        self.map_points = ['/point1', '/point2'] # pos on map

    def execute(self, userdata):
        # wonder between points
        index = 0
        while not rospy.is_shutdown():
            if self.preempt_requested():
                # cancel the current base goal
                # \TODO: check if there is a process
                self.controllers.base_controller.cancel()
                rospy.logwarn(' Preempting the Explore Surroundings state')
                self.service_preempt()
                return 'finished_exploring'
            if not is_running(self.controllers.base_controller.get_client()):
                print('i am here', index)
                self.controllers.base_controller.async_to_pose(get_pose_from_param(self.map_points[index]))
                if index == 1:
                    index = 0
                else:
                    index = 1


if __name__ == '__main__':
    rospy.init_node('smach_example_state_machine')
    con = Controllers()
    sm = smach.StateMachine(outcomes=['finished_exploring'])
    with sm:
        smach.StateMachine.add('EXPLORE_SURROUNDINGS', ExploreSurroundingsState(con),
                               transitions={'finished_exploring':'finished_exploring'},)
    outcome = sm.execute()
    rospy.loginfo('I have completed execution with outcome: ')
    rospy.loginfo(outcome)
