import rospy
import actionlib
from unsafe_traversal.srv import ChangeTraversalParameters
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from unsafe_traversal.msg import MoveToGoalAction, AlignToGoalAction, MoveToGoalGoal, AlignToGoalGoal, MoveToGoalResult, AlignToGoalResult
from sensor_msgs.msg import LaserScan
import numpy as np

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
        print("in")
        result = MoveToGoalResult()

        align_poses(msg.start_pose, msg.end_pose)
        print('align poses')
        self.move(msg.start_pose)
        print('at start pose')

        rospy.sleep(3)

        laser_scan = rospy.wait_for_message("/scan", LaserScan)
        filtered_ranges = laser_scan.ranges[len(laser_scan.ranges)//3 : 2*len(laser_scan.ranges)//3]
        mean_distance = np.nanmean(filtered_ranges)
        print(mean_distance)
        if mean_distance < 2.0:
            result.success = False
            self._move_server.set_aborted(result)
            return result

        self._proxy(True)
        print('after proxy true')
        self.move(msg.end_pose)
        print('move to end')
        self._proxy(False)
        print('after proxy false')

        result.success = True
        self._move_server.set_succeeded(result)

        return result
