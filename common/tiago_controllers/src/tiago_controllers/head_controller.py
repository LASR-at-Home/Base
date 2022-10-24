#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from tiago_controllers.helpers import get_joint_values, is_running


class HeadController:
    def __init__(self):
        self._client = actionlib.SimpleActionClient("/head_controller/follow_joint_trajectory",
                                                    FollowJointTrajectoryAction)
        self._client.wait_for_server()

    def get_client(self):
        return self._client

    def is_running(self):
        return is_running(self._client)

    def __go_to_position(self, joint1, joint2, time_from_start=1, velocities=None):
        """
        Args:
            :param velocities: list[float64, float64]
            :param joint1: sideways rotation
            :param joint2: up/down rotation
            :param time_from_start: time from start
        """
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [joint1, joint2]
        if velocities is not None:
            point.velocities = velocities
        point.time_from_start = rospy.Duration(time_from_start)
        goal.trajectory.points.append(point)

        self._client.send_goal(goal)

    def async_reach_to(self, joint1, joint2, time_from_start=1, velocities=None):
        self.__go_to_position(joint1, joint2, time_from_start, velocities)

    def sync_reach_to(self, joint1, joint2, time_from_start=1, wait=10, velocities=None):
        self.__go_to_position(joint1, joint2, time_from_start, velocities)
        done = self._client.wait_for_result(rospy.Duration(wait))
        state = self._client.get_state()
        return done and state == actionlib.GoalStatus.SUCCEEDED

    @staticmethod
    def get_joint_values():
        return get_joint_values("/head_controller/query_state")


if __name__ == '__main__':
    rospy.init_node("head_test", anonymous=True)
    _head = HeadController()
    _head.sync_reach_to(0, 0)
