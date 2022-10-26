#!/usr/bin/env python3

import actionlib
import rospy
from tiago_controllers.helpers import is_running, is_terminated
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


class BaseController:
    def __init__(self):
        self._goal_sent = False
        self._client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self._client.wait_for_server()

    def cancel_goal(self):
        if self._goal_sent is True:
            state = self._client.get_state()
            if state == GoalStatus.PENDING or state == GoalStatus.ACTIVE:
                self._client.cancel_goal()
            while True:
                if (self._client.get_state() in [GoalStatus.PREEMPTED, GoalStatus.ABORTED, GoalStatus.REJECTED,
                                                 GoalStatus.RECALLED, GoalStatus.SUCCEEDED]):
                    break
                rospy.sleep(0.5)
                self._goal_sent = False

    def is_running(self):
        return is_running(self._client)

    def get_status(self):
        return self._client.get_state()

    def is_active(self):
        return self._client.get_state() == GoalStatus.PENDING or self._client.get_state() == GoalStatus.ACTIVE

    def is_terminated(self):
        return is_terminated(self._client)

    def __to_pose(self, pose, done_cb=None):
        if pose:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = pose

            rospy.loginfo("base is going to (%.2f, %.2f, %.2f) pose", pose.position.x, pose.position.y, pose.position.z)

            self._goal_sent = True
            self._client.send_goal(goal, done_cb=done_cb)

    def async_to_pose(self, pose):
        self.__to_pose(pose)

    def get_client(self):
        return self._client

    def sync_to_pose(self, pose, wait=60):
        self.__to_pose(pose)
        done = self._client.wait_for_result(rospy.Duration(wait))
        self._goal_sent = False

        state = self._client.get_state()
        if done and state == GoalStatus.SUCCEEDED:
            return True
        return state
