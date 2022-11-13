#!/usr/bin/env python3

import sys
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

from robocup_receptionist.utils import activate_robot_navigation, clear_costmap
from dialogflow_speech.utils import talk


class GoToStart(smach.State):
    def __init__(self, base_controller, head_controller, torso_controller):
        smach.State.__init__(self, outcomes=['succeeded'])
        if rospy.has_param('/entrance'):
            pose = rospy.get_param('/entrance')
        else:
            pose = {
                'position' : {
                    'x' : 0.81,
                    'y' : -2.16,
                    'z' : 0
                },
                'orientation' : {
                    'x' : 0,
                    'y' : 0,
                    'z' : -0.99,
                    'w' : 0.05
                }
            }
        self.goal = Pose(Point(pose['position']['x'], 
                               pose['position']['y'], 
                               pose['position']['z']), 
                        Quaternion(pose['orientation']['x'], 
                                   pose['orientation']['y'], 
                                   pose['orientation']['z'], 
                                   pose['orientation']['w']))
            
        self.base_controller = base_controller
        self.head_controller = head_controller
        self.torso_controller = torso_controller

    def execute(self, ud):
        activate_robot_navigation(True)
        clear_costmap()
        talk("I'm going to the entrance.")
        rospy.loginfo(f"sending pose goal \n {self.goal.position} to base_controller...")
        success = self.base_controller.sync_to_pose(self.goal)
        activate_robot_navigation(False)
        self.head_controller.async_reach_to(0.0, 0.0)
        self.torso_controller.sync_reach_to(0.15)
        if success == True:
            talk("Please open the door and close it behind you. You'll get more instructions shortly!")
        else:
            rospy.loginfo(f"failed pose goal \n {self.goal.position} to base_controller, status {success}")
            self.base_controller.sync_to_radius(self.goal.position.point.x, self.goal.position.point.y)
            if rospy.has_param('/door'):
                self.base_controller.sync_face_to(self.goal.position.point.x, self.goal.position.point.y)
        return 'succeeded'

# class GoToEntranceAndWaitSM(smach.StateMachine):
#     def __init__(self, base_controller, head_controller):
#         smach.StateMachine.__init__(self, outcomes=['human_in_range', 'aborted'], output_keys=["person_bb"])


#         with self:

#             #! Go to entrance
#             smach.StateMachine.add('GO_TO_ENTRANCE', 
#                                     GoToStart(base_controller), 
#                                     transitions={
#                                                     'succeeded': 'WAIT_FOR_PERSON', 
#                                                     'failed': 'aborted'
#                                                 })

#             #! wait for person
#             smach.StateMachine.add('WAIT_FOR_PERSON', 
#                                     WaitForGuestState(base_controller, head_controller), 
#                                     transitions={
#                                                     'human_in_range': 'human_in_range',
#                                                     'not_human_in_range':'WAIT_FOR_PERSON'
#                                                 },
#                                     remapping = {
#                                         "person_bb" : "person_bb"
#                                     })
