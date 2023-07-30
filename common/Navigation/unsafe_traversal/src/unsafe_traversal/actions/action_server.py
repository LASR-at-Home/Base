#!/usr/bin/env python3
import rospy
import actionlib
from unsafe_traversal.srv import ChangeTraversalParameters, DeterminePathViability, LaserDist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from unsafe_traversal.msg import MoveToGoalAction, AlignToGoalAction, MoveToGoalGoal, AlignToGoalGoal, MoveToGoalResult, AlignToGoalResult
from sensor_msgs.msg import LaserScan
import numpy as np
from common_math import euclidian

from ..quaternion import align_poses
from .move_base_client import MoveBaseClient


class TraversalActionServer(MoveBaseClient):
    '''
    Action server for moving to a goal with unsafe traversal enabled.
    '''

    _proxy = rospy.ServiceProxy(
        '/unsafe_traversal/set_unsafe_traversal', ChangeTraversalParameters)

    def __init__(self):
        # create action servers
        self._align_server = actionlib.SimpleActionServer(
            '/unsafe_traversal/align_to_goal', AlignToGoalAction, execute_cb=self.align_action_cb, auto_start=False)
        self._move_server = actionlib.SimpleActionServer(
            '/unsafe_traversal/move_to_goal', MoveToGoalAction, execute_cb=self.move_action_cb, auto_start=False)

        # start the servers
        self._align_server.start()
        self._move_server.start()

        self.viable_plan_srv = rospy.ServiceProxy("/unsafe_traversal/check_if_plan_is_viable", DeterminePathViability)
        self.laser_dist_srv = rospy.ServiceProxy("/unsafe_traversal/laser_dist_checker", LaserDist)



    def align_action_cb(self, msg):
        '''
        Align action server callback
        '''

        align_poses(msg.start_pose, msg.end_pose)
        self.move(msg.start_pose)

        self._align_server.set_succeeded(AlignToGoalResult())

    def move_action_cb(self, msg):
        '''
        Move action server callback
        '''
        result = MoveToGoalResult()

        align_poses(msg.start_pose, msg.end_pose)
        self.move(msg.start_pose)

        viable = self.viable_plan_srv(msg.start_pose, msg.end_pose).viable
        laser_dist = self.laser_dist_srv(60.).dist
        print(viable, laser_dist)
        if not viable and laser_dist < 1.5:
            result.success = False
            self._move_server.set_aborted(result)
            return result

        self._proxy(True)
        self.move(msg.end_pose)
        self._proxy(False)

        result.success = True
        self._move_server.set_succeeded(result)

        return result
