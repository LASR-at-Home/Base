"""
place_state.py — YASMIN state: release the held laundry onto the desk.

Assumes Move already brought the arm above the desk place spot. Place then:
    1) lower    : straight down by `lower` metres (set cloth on surface)
    2) open     : open gripper (release)
    3) retreat  : straight up by `retreat` metres (clear the cloth)

Give the place spot as a base-frame pose (same convention as Move), or omit it
to release in place at the current commanded spot.

Confirm group_name / base_link / ee_link / gripper joints, and run the node
under a MultiThreadedExecutor (pymoveit2 needs it).
"""

import math
import time

from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

try:
    from yasmin import State
except ImportError:
    class State:
        def __init__(self, outcomes):
            self._outcomes = outcomes


def _top_down_quat(yaw):
    half = yaw / 2.0
    x1, y1, z1, w1 = 0.0, 0.0, math.sin(half), math.cos(half)
    x2, y2, z2, w2 = 0.0, 0.7071068, 0.0, 0.7071068
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


ARM_TORSO_JOINTS = ['torso_lift_joint', 'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
                    'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']


class Place(State):
    def __init__(self, x, y, z, yaw=0.0,
                 lower=0.10, retreat=0.20, open_pos=0.044,
                 cartesian=True,
                 group_name='arm_torso', base_link='base_footprint',
                 ee_link='gripper_grasping_frame', joint_names=None,
                 gripper_action='/gripper_controller/follow_joint_trajectory',
                 gripper_joints=('gripper_left_finger_joint', 'gripper_right_finger_joint')):
        super().__init__(outcomes=['succeeded', 'failed'])
        self.x, self.y, self.z, self.yaw = x, y, z, yaw
        self.lower, self.retreat = lower, retreat
        self.open_pos = open_pos
        self.cartesian = cartesian
        self.gripper_joints = gripper_joints

        from yasmin_ros.yasmin_node import YasminNode
        from pymoveit2 import MoveIt2
        self.node = YasminNode.get_instance()
        cb = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self.node,
            joint_names=joint_names or ARM_TORSO_JOINTS,
            base_link_name=base_link,
            end_effector_name=ee_link,
            group_name=group_name,
            callback_group=cb)
        self.grip_pub = self.node.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)

    def _wait(self, future, timeout=15.0):
        t0 = time.time()
        while not future.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        return future.result() if future.done() else None

    def _move(self, xyz, cartesian):
        try:
            self.moveit2.move_to_pose(
                position=[float(xyz[0]), float(xyz[1]), float(xyz[2])],
                quat_xyzw=_top_down_quat(self.yaw), cartesian=cartesian)
            self.moveit2.wait_until_executed()
            return True
        except Exception as e:
            self.node.get_logger().error(f'[Place] move failed: {e}')
            return False

    def _open(self):
        msg = JointTrajectory(
            joint_names=list(self.gripper_joints),
            points=[JointTrajectoryPoint(
                positions=[float(self.open_pos)] * len(self.gripper_joints),
                time_from_start=Duration(sec=1))])
        for _ in range(3):
            self.grip_pub.publish(msg)
            time.sleep(0.1)
        time.sleep(1.5)
        return True

    def execute(self, blackboard):
        log = self.node.get_logger()
        down = (self.x, self.y, self.z - self.lower)
        up = (self.x, self.y, self.z + self.retreat)

        log.info('[Place] lower onto desk')
        if not self._move(down, self.cartesian):
            return 'failed'
        log.info('[Place] open gripper (release)')
        if not self._open():
            return 'failed'
        log.info('[Place] retreat up')
        if not self._move(up, self.cartesian):
            return 'failed'
        return 'succeeded'