"""
move.py — YASMIN state: carry the held laundry to a fixed place pose
(e.g. above the desk) via MoveIt2. Gripper left as-is (keeps holding).

  Move(x=0.6, y=-0.3, z=0.95, yaw=0.0)
"""

import math

from rclpy.callback_groups import ReentrantCallbackGroup

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


class Move(State):
    def __init__(self, x, y, z, yaw=0.0, quat_xyzw=None, cartesian=False,
                 group_name='arm_torso', base_link='base_footprint',
                 ee_link='gripper_grasping_frame', joint_names=None):
        super().__init__(outcomes=['succeeded', 'failed'])
        self.x, self.y, self.z, self.yaw = x, y, z, yaw
        self.quat_xyzw = quat_xyzw
        self.cartesian = cartesian

        from yasmin_ros.yasmin_node import YasminNode
        from pymoveit2 import MoveIt2
        self.node = YasminNode.get_instance()
        self.moveit2 = MoveIt2(
            node=self.node,
            joint_names=joint_names or ARM_TORSO_JOINTS,
            base_link_name=base_link,
            end_effector_name=ee_link,
            group_name=group_name,
            callback_group=ReentrantCallbackGroup())

    def execute(self, blackboard):
        quat = self.quat_xyzw or _top_down_quat(self.yaw)
        try:
            self.moveit2.move_to_pose(
                position=[float(self.x), float(self.y), float(self.z)],
                quat_xyzw=quat, cartesian=self.cartesian)
            self.moveit2.wait_until_executed()
            self.node.get_logger().info(f'[Move] reached ({self.x}, {self.y}, {self.z})')
            return 'succeeded'
        except Exception as e:
            self.node.get_logger().error(f'[Move] move failed: {e}')
            return 'failed'
