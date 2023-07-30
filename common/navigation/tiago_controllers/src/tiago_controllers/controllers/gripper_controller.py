#!/usr/bin/env python3
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from tiago_controllers.helpers import get_joint_values, is_running, cancel_goal


class GripperController:
    JOINT_MAX = 0.045

    def __init__(self):
        self._client = actionlib.SimpleActionClient("gripper_controller/follow_joint_trajectory",
                                                    FollowJointTrajectoryAction)
        self._client.wait_for_server()

    def sync_reach_to(self, joint1, joint2=None, time_from_start=1):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
        point = JointTrajectoryPoint()
        if not joint2:
            joint2 = joint1
        point.positions = [joint1, joint2]
        point.time_from_start = rospy.Duration(time_from_start)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)
        done = self._client.wait_for_result()
        state = self._client.get_state()
        if done and state == actionlib.GoalStatus.SUCCEEDED:
            return True
        return state

    def cancel_goal(self):
        return cancel_goal(self, '/torso_controller/follow_joint_trajectory/cancel', self._client)

    def is_running(self):
        return is_running(self._client)

    def get_gripper_state(self):
        return get_joint_values('/gripper_controller/query_state')
