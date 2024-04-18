#!/usr/bin/env python3

import math
import time
import actionlib
import rospy
from tiago_controllers.helpers import is_running, is_terminated
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from tiago_controllers.base_planner import plan_to_radius as _plan_to_radius
from math import atan2, radians
from geometry_msgs.msg import Twist
from common_math.transformations import quaternion_from_euler
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from scipy.spatial.transform import Rotation as R
from tiago_controllers.base_planner import get_journey_points as _get_journey_points
import numpy as np
import numpy as np

class BaseController:
    def __init__(self):
        self._goal_sent = False
        self.__client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.__client.wait_for_server()

    def get_pose(self):
        return self.get_current_pose()[:2]

    def cancel_goal(self):
        if self._goal_sent is True:
            state = self.__client.get_state()
            if state == GoalStatus.PENDING or state == GoalStatus.ACTIVE:
                self.__client.cancel_goal()
            while True:
                if (self.__client.get_state() in [GoalStatus.PREEMPTED, GoalStatus.ABORTED, GoalStatus.REJECTED,
                                                  GoalStatus.RECALLED, GoalStatus.SUCCEEDED]):
                    break
                rospy.sleep(0.5)
                self._goal_sent = False

    def check_active_state(self):
        return self._client.get_state() == GoalStatus.PENDING or self._client.get_state() == GoalStatus.ACTIVE

    def check_terminated_state(self):
        return self._client.get_state() == GoalStatus.LOST or self._client.get_state() == GoalStatus.PREEMPTED or \
               self._client.get_state() == GoalStatus.ABORTED

    def get_current_pose(self):
        msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        x = round(msg.pose.pose.position.x, 2)
        y = round(msg.pose.pose.position.y, 2)
        quat = msg.pose.pose.orientation
        return x, y, quat

    def is_running(self):
        return is_running(self.__client)

    def reg_callback(self, callback):
        self._callback = callback

    def get_status(self):
        return self.__client.get_state()

    def is_active(self):
        return self.__client.get_state() == GoalStatus.PENDING or self.__client.get_state() == GoalStatus.ACTIVE

    def is_terminated(self):
        return is_terminated(self.__client)

    def __to_pose(self, pose, done_cb=None):
        if pose:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = pose

            rospy.loginfo("base is going to (%.2f, %.2f, %.2f) pose", pose.position.x, pose.position.y, pose.position.z)

            self._goal_sent = True
            self.__client.send_goal(goal, done_cb=done_cb)

    def async_to_pose(self, pose):
        self.__to_pose(pose)

    def get_client(self):
        return self.__client

    def sync_to_pose(self, pose, wait=60):
        self.__to_pose(pose)
        done = self.__client.wait_for_result(rospy.Duration(wait))
        self._goal_sent = False

        state = self.__client.get_state()
        if done and state == GoalStatus.SUCCEEDED:
            return True
        return state

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

    def ensure_sync_to_pose(self, pose, wait=60):
        state = self.sync_to_pose(pose)
        print("State of the robot is {}".format(state))
        while state == 8 or state == 4 or state == 0:
            rospy.loginfo("Retrying to reach pose - prev state is {}".format(state))
            rospy.sleep(0.5)
            state = self.sync_to_pose(pose)
        return state

    def rotate_to_face_object(self, object_name='/door/pose'):
        object_pos = get_pose_from_param(object_name)
        rotation_angle = self.compute_face_quat(object_pos.position.x, object_pos.position.y)
        state = self.sync_to_pose(rotation_angle)

        return state

    # to be tested more remove if unnecessary
    # def calculate_angle_of_rot(self, robot_pose, door_position):
    #     delta_x = door_position.position.x - robot_pose.position.x
    #     delta_y = door_position.position.y - robot_pose.position.y
    #     angle_to_door = math.atan2(delta_y, delta_x)
    #
    #     if angle_to_door > math.pi / 2:
    #         angle_to_door -= math.pi
    #     elif angle_to_door < -math.pi / 2:
    #         angle_to_door += math.pi
    #
    #     (x, y, z, w) = quaternion_from_euler(0, 0, angle_to_door)
    #     quaternion = Quaternion(x, y, z, w)
    #     pose = Pose(position=Point(robot_pose.position.x, robot_pose.position.y, 0.0), orientation=quaternion)
    #     return pose

    def rotate(self, radians):
        x, y, current_orientation = self.get_current_pose()
        current_orientation = np.array([current_orientation.x, current_orientation.y,
                                        current_orientation.z, current_orientation.w])
        r = R.from_quat(current_orientation)
        rotated_r = r * R.from_euler('z', radians, degrees=False)

        pose = Pose(position=Point(x, y, 0.0), orientation=Quaternion(*rotated_r.as_quat()))

        return self.sync_to_pose(pose)


class CmdVelController:
    def __init__(self):
        self._vel_pub = rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size=1)
        # rospy.sleep(0.1)  # wait for publisher to activate

    def rotate(self, angular_velocity, angle, clockwise):
        vel_msg = Twist()
        angular_velocity = radians(angular_velocity)
        angle = radians(angle)
        if clockwise:
            vel_msg.angular.z = -abs(angular_velocity)
        else:
            vel_msg.angular.z = abs(angular_velocity)

        curr_ang = 0.0
        t0 = rospy.Time.now().to_sec()

        while curr_ang < angle:
            self._vel_pub.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            curr_ang = angular_velocity * (t1 - t0)

    def linear_movement(self, speed, distance, is_forward):
        vel_msg = Twist()
        if is_forward:
            vel_msg.linear.x = abs(speed)
        else:
            vel_msg.linear.x = -abs(speed)
        curr_dist = 0
        t0 = rospy.Time.now().to_sec()
        while curr_dist < distance:
            t1 = rospy.Time.now().to_sec()
            curr_dist = curr_dist + speed * (t1 - t0)
            self._vel_pub.publish(vel_msg)


class ReachToRadius(BaseController):
    def __init__(self):
        super().__init__()

    def sync_to_radius(self, x, y, radius=0.5, tol=0.1):
        points = self.plan_to_radius(x, y, radius, tol)
        print(points)
        for point in points:
            print(point)
            result = self.sync_to_pose(point)
            if result:
                return result

    def plan_to_radius(self, x, y, radius=0.5, tol=0.1):
        robot_x, robot_y, quaternion = self.get_current_pose()

        return _plan_to_radius(
            Pose(
                position=Point(robot_x, robot_y, 0.0),
                orientation=quaternion
            ),
            (x, y), radius, tol
        )

    def sync_face_to(self, x, y):
        return self.sync_to_pose(self.compute_face_quat(x, y))

    def compute_face_quat(self, x, y):
        robot_x, robot_y, quat = self.get_current_pose()

        dist_x = x - robot_x
        dist_y = y - robot_y
        angle = atan2(dist_y, dist_x)

        (x, y, z, w) = quaternion_from_euler(0, 0, angle)
        quaternion = Quaternion(x, y, z, w)

        pose = Pose(position=Point(robot_x, robot_y, 0.0), orientation=quaternion)
        return pose



if __name__ == '__main__':
    rospy.init_node("base_test", anonymous=True)
    b = BaseController()
    b.sync_face_to(2.84, 6.7)
