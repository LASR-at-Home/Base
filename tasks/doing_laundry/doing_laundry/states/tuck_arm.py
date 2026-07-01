"""
tuck_arm.py — YASMIN state: move the arm to a folded/home joint configuration
so it is out of the camera view before detection (and safe for navigation).

Uses a joint-space goal (not a pose), which planned reliably in testing.

  TuckArm()                        # default tuck config
  TuckArm(config=[...])            # 8 values: torso + arm_1..7

Confirm the config is collision-free for your robot; tune if needed.
"""

import time

from rclpy.callback_groups import ReentrantCallbackGroup

try:
    from yasmin import State
except ImportError:
    class State:
        def __init__(self, outcomes):
            self._outcomes = outcomes


ARM_TORSO_JOINTS = ['torso_lift_joint', 'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
                    'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']

# torso, arm_1..7 — a tucked pose that keeps the arm low and clear of the head cam.
DEFAULT_TUCK = [0.15, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.00]


class TuckArm(State):
    def __init__(self, config=None,
                 group_name='arm_torso', base_link='base_footprint',
                 ee_link='gripper_grasping_frame', joint_names=None):
        super().__init__(outcomes=['succeeded', 'failed'])
        self.config = list(config) if config else list(DEFAULT_TUCK)
        from yasmin_ros.yasmin_node import YasminNode
        from pymoveit2 import MoveIt2
        self.node = YasminNode.get_instance()
        self.moveit2 = MoveIt2(
            node=self.node, joint_names=joint_names or ARM_TORSO_JOINTS,
            base_link_name=base_link, end_effector_name=ee_link,
            group_name=group_name, callback_group=ReentrantCallbackGroup())

    def execute(self, blackboard):
        try:
            self.node.get_logger().info('[TuckArm] moving arm to home/tuck')
            self.moveit2.move_to_configuration(self.config)
            ok = self.moveit2.wait_until_executed()
            if ok is False:
                self.node.get_logger().error('[TuckArm] planning/exec FAILED')
                return 'failed'
            return 'succeeded'
        except Exception as e:
            self.node.get_logger().error(f'[TuckArm] error: {e}')
            return 'failed'
