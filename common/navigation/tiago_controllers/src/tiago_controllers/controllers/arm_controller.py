#!/usr/bin/env python
from __future__ import print_function

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from actionlib_msgs.msg import GoalStatus
from tiago_controllers.helpers import get_joint_values, is_running, cancel_goal


class ArmController:
    def __init__(self):
        self._client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory',
                                                    FollowJointTrajectoryAction)
        self._client.wait_for_server()
        print("done waiting")

    def _is_running(self):
        return self._client.get_state() == GoalStatus.PENDING or self._client.get_state() == GoalStatus.ACTIVE

    def sync_reach_to(self, joint1, join2, joint3, joint4, joint5, joint6, joint7, time_from_start=1):
        """
        Args:
            :param joint#: new position of joint
            :param time_from_start: time from start
        """
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint',
                                       'arm_6_joint', 'arm_7_joint']
        points = JointTrajectoryPoint()
        points.positions = [joint1, join2, joint3, joint4, joint5, joint6, joint7]
        points.time_from_start = rospy.Duration(time_from_start)
        goal.trajectory.points.append(points)
        self._client.send_goal(goal)
        done = self._client.wait_for_result()
        state = self._client.get_state()
        print(state)
        return done and state

    def cancel_goal(self):
        return cancel_goal(self, '/arm_controller/follow_joint_trajectory/cancel', self._client)

    def get_client(self):
        return self._client

    def is_running(self):
        return _is_running(self._client)

    @staticmethod
    def get_joint_values():
        return get_joint_values("/arm_controller/query_state")