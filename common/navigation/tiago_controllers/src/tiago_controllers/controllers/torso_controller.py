#!/usr/bin/env python3

from __future__ import print_function
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint
from tiago_controllers.helpers import get_joint_values, is_running, cancel_goal

# TODO: refactor sync and async

class TorsoController:
    JOINT_MAX = 0.35

    def __init__(self):
        self._client = actionlib.SimpleActionClient("torso_controller/follow_joint_trajectory",
                                                    FollowJointTrajectoryAction)
        self._client.wait_for_server()

    def get_client(self):
        return self._client

    def sync_reach_to(self, joint1):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["torso_lift_joint"]
        point = JointTrajectoryPoint()
        point.positions = [joint1]
        point.time_from_start = rospy.Duration(1)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)
        self._client.wait_for_result()
        return True

    def async_reach_to(self, joint1):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["torso_lift_joint"]
        point = JointTrajectoryPoint()
        point.positions = [joint1]
        point.time_from_start = rospy.Duration(1)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)

    def cancel_goal(self):
        return cancel_goal(self, '/torso_controller/follow_joint_trajectory/cancel', self._client)

    def is_running(self):
        return is_running(self._client)

    @staticmethod
    def get_joint_values():
        return get_joint_values("/torso_controller/query_state")

