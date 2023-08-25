#!/usr/bin/env python3

# from abc import abstractmethod
import math
import time
import actionlib
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from helpers.transformations import quaternion_from_euler, euler_from_quaternion
from actionlib_msgs.msg import GoalStatus
from tiago_controllers.base_planner import plan_to_radius as _plan_to_radius
from tiago_controllers.base_planner import get_journey_points as _get_journey_points
import numpy as np
from scipy.spatial.transform import Rotation as R

class BaseController:
    def __init__(self):
        self._goal_sent = False
        self._callback = None
        self._client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self._velocity_publisher = rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size=10)
        self._client.wait_for_server()

    def get_pose(self):
        msg = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
        x = round(msg.pose.pose.position.x, 2)
        y = round(msg.pose.pose.position.y, 2)
        return x, y


    def reg_callback(self, callback):
        self._callback = callback

    def get_status(self):
        return self._client.get_state()

    def get_middle_journey(self, x, y, tol=0.01):
        robot_x, robot_y, quaternion = self.get_current_pose()
        return _get_journey_points(
            Pose(
                position=Point(robot_x, robot_y, 0.0),
                orientation=quaternion
            ), x, y, tol
        )

    def check_active_state(self):
        return self._client.get_state() == GoalStatus.PENDING or self._client.get_state() == GoalStatus.ACTIVE

    def check_terminated_state(self):
        return self._client.get_state() == GoalStatus.LOST or self._client.get_state() == GoalStatus.PREEMPTED or \
               self._client.get_state() == GoalStatus.ABORTED

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

    def get_current_pose(self):
        msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        x = round(msg.pose.pose.position.x, 2)
        y = round(msg.pose.pose.position.y, 2)
        quat = msg.pose.pose.orientation
        return x, y, quat

    def plan_to_radius(self, x, y, radius=0.5, tol=0.1):
        robot_x, robot_y, quaternion = self.get_current_pose()

        return _plan_to_radius(
            Pose(
                position=Point(robot_x, robot_y, 0.0),
                orientation=quaternion
            ),
            (x, y), radius, tol
        )

    def sync_to_radius(self, x, y, radius=0.5, tol=0.1):
        points = self.plan_to_radius(x, y, radius, tol)
        if points:
            for point in points:
                result = self.sync_to_pose(point)
                if result:
                    return result
        else:
            rospy.loginfo(" no points selected")

    def cancel(self):
        if self._goal_sent is True:
            state = self._client.get_state()
            if state == GoalStatus.PENDING or state == GoalStatus.ACTIVE:
                self._client.cancel_goal()
            while True:
                if (self._client.get_state() in [GoalStatus.PREEMPTED, GoalStatus.ABORTED, GoalStatus.REJECTED,
                                                 GoalStatus.RECALLED, GoalStatus.SUCCEEDED]):
                    break
                time.sleep(0.5)
                self._goal_sent = False

    def compute_face_quat(self, x, y):
        robot_x, robot_y, robot_quat = self.get_current_pose()
        dist_x = x - robot_x
        dist_y = y - robot_y
        theta_deg = np.degrees(math.atan2(dist_y, dist_x))
        try:
            from scipy.spatial.transform import Rotation as R
            (x, y, z, w) = R.from_euler("z", theta_deg, degrees=True).as_quat()
            quaternion = Quaternion(x, y, z, w)
        except ImportError:
            quaternion = robot_quat
        pose = Pose(position=Point(robot_x, robot_y, 0.0), orientation=quaternion)
        return pose

    def sync_face_to(self, x, y):
        return self.sync_to_pose(self.compute_face_quat(x, y))

    def async_face_to(self, x, y):
        return self.async_to_pose(self.compute_face_quat(x, y))

    def rotate(self, radians):
        x, y, current_orientation = self.get_current_pose()
        current_orientation = np.array([current_orientation.x, current_orientation.orientation.y,
                                        current_orientation.z, current_orientation.w])
        r = R.from_quat(current_orientation)
        rotated_r = r * R.from_euler('z', radians, degrees=False)

        pose = Pose(position=Point(x, y, 0.0), orientation=Quaternion(**rotated_r.as_quat()))

        return self.sync_to_pose(pose)


if __name__ == '__main__':
    rospy.init_node("base_test", anonymous=True)
    b = BaseController()
    b.sync_face_to(2.84, 6.7)