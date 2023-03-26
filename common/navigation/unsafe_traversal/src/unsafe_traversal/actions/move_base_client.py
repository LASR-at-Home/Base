#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class MoveBaseClient:
    '''
    Configure a client for move_base
    '''

    _move_base_client = actionlib.SimpleActionClient(
        'move_base', MoveBaseAction)

    def __init__(self):
        self._move_base_client.wait_for_server()

    def move(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose = pose

        self._move_base_client.send_goal(goal)
        self._move_base_client.wait_for_result()
