"""
pick.py — YASMIN state: pick one T-shirt.

Collision is PERCEPTION-DRIVEN: the basket walls are added to the MoveIt
planning scene in real time from the blackboard pose that Detect wrote
(basket_pose + basket_yaw). No hard-coded basket location. MoveIt then plans
BOTH the approach and the descent around those walls, so the arm neither
swings through the basket on the way in nor on the way down.

Walls are an OPEN box (4 sides + bottom, no lid) so a top-down gripper can
enter the opening. Descent is done in small vertical steps for robustness.
Walls are removed at the end (success or failure).

Reads blackboard (Detect 'grasp_ready'):
    basket_pose (cx,cy,rim_z), basket_yaw, grasp_yaw,
    pre_grasp_pose, grasp_pose, lift_pose
"""

import math
import time

from rclpy.callback_groups import ReentrantCallbackGroup
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

try:
    from yasmin import State
except ImportError:
    class State:
        def __init__(self, outcomes):
            self._outcomes = outcomes


OPEN_X, OPEN_Y, BASKET_H = 0.364, 0.362, 0.31
WALL_T = 0.02

ARM_TORSO_JOINTS = ['torso_lift_joint', 'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
                    'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']


def _top_down_quat(yaw):
    hy = yaw / 2.0
    x1, y1, z1, w1 = 0.0, 0.0, math.sin(hy), math.cos(hy)
    x2, y2, z2, w2 = 0.0, 0.7071068, 0.0, 0.7071068
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


class Pick(State):
    def __init__(self,
                 group_name='arm_torso', base_link='base_footprint',
                 ee_link='gripper_grasping_frame', joint_names=None,
                 gripper_topic='/gripper_controller/joint_trajectory',
                 gripper_joints=('gripper_left_finger_joint', 'gripper_right_finger_joint'),
                 open_pos=0.044, close_pos=0.0,
                 floor_stop=0.02, step=0.05,
                 wall_shrink=0.0,           # shrink walls inward (m) if padding blocks entry
                 add_walls=True,
                 rim_z_min=0.15, rim_z_max=0.45):
        super().__init__(outcomes=['succeeded', 'failed'])
        self.gripper_joints = gripper_joints
        self.open_pos, self.close_pos = open_pos, close_pos
        self.floor_stop, self.step = floor_stop, step
        self.wall_shrink = wall_shrink
        self.add_walls = add_walls
        self.rim_z_min, self.rim_z_max = rim_z_min, rim_z_max
        self.base_link = base_link

        from yasmin_ros.yasmin_node import YasminNode
        from pymoveit2 import MoveIt2
        self.node = YasminNode.get_instance()
        cb = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self.node, joint_names=joint_names or ARM_TORSO_JOINTS,
            base_link_name=base_link, end_effector_name=ee_link,
            group_name=group_name, callback_group=cb)
        self.grip_pub = self.node.create_publisher(JointTrajectory, gripper_topic, 10)
        self._wall_ids = []

    # ---- gripper ----
    def _gripper(self, pos):
        msg = JointTrajectory(
            joint_names=list(self.gripper_joints),
            points=[JointTrajectoryPoint(
                positions=[float(pos)] * len(self.gripper_joints),
                time_from_start=Duration(sec=1))])
        for _ in range(3):
            self.grip_pub.publish(msg)
            time.sleep(0.1)
        time.sleep(1.5)
        return True

    # ---- motion ----
    def _move(self, xyz, yaw):
        try:
            self.node.get_logger().info(
                f'[Pick] move -> ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})')
            self.moveit2.move_to_pose(
                position=[float(xyz[0]), float(xyz[1]), float(xyz[2])],
                quat_xyzw=_top_down_quat(yaw), cartesian=False)
            ok = self.moveit2.wait_until_executed()
            if ok is False:
                self.node.get_logger().error('[Pick] move FAILED')
                return False
            return True
        except Exception as e:
            self.node.get_logger().error(f'[Pick] move error: {e}')
            return False

    # ---- perception-driven walls ----
    def _add_basket(self, cx, cy, rim_z, yaw):
        if not self.add_walls:
            return
        floor = rim_z - BASKET_H
        c, s = math.cos(yaw), math.sin(yaw)
        q = [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]
        sh = self.wall_shrink
        ox, oy = OPEN_X - sh, OPEN_Y - sh
        hx, hy = ox / 2 + WALL_T / 2, oy / 2 + WALL_T / 2
        boxes = [
            ('basket_xp', (hx, 0.0), (WALL_T, oy + 2 * WALL_T, BASKET_H), rim_z - BASKET_H / 2),
            ('basket_xn', (-hx, 0.0), (WALL_T, oy + 2 * WALL_T, BASKET_H), rim_z - BASKET_H / 2),
            ('basket_yp', (0.0, hy), (ox + 2 * WALL_T, WALL_T, BASKET_H), rim_z - BASKET_H / 2),
            ('basket_yn', (0.0, -hy), (ox + 2 * WALL_T, WALL_T, BASKET_H), rim_z - BASKET_H / 2),
            ('basket_bot', (0.0, 0.0), (ox + 2 * WALL_T, oy + 2 * WALL_T, WALL_T), floor + WALL_T / 2),
        ]
        for name, (lx, ly), size, wz in boxes:
            wx = cx + lx * c - ly * s
            wy = cy + lx * s + ly * c
            try:
                self.moveit2.add_collision_box(
                    id=name, size=list(size), position=[wx, wy, wz],
                    quat_xyzw=q, frame_id=self.base_link)
                self._wall_ids.append(name)
            except Exception as e:
                self.node.get_logger().warn(f'[Pick] wall {name} failed: {e}')
        time.sleep(0.6)  # let planning scene update before planning
        self.node.get_logger().info(f'[Pick] basket walls added ({len(self._wall_ids)})')

    def _remove_basket(self):
        for name in self._wall_ids:
            try:
                self.moveit2.remove_collision_object(name)
            except Exception:
                pass
        self._wall_ids = []
        time.sleep(0.3)

    # ---- main ----
    def execute(self, blackboard):
        log = self.node.get_logger()
        try:
            bcx, bcy, brim = blackboard['basket_pose']
            byaw = blackboard['basket_yaw']
            yaw = blackboard['grasp_yaw']
            pre = blackboard['pre_grasp_pose']
            grasp = blackboard['grasp_pose']
            lift = blackboard['lift_pose']
        except Exception as e:
            log.error(f'[Pick] blackboard read failed: {e}')
            return 'failed'

        gx, gy, gz = grasp
        floor = brim - BASKET_H
        gz = max(gz, floor + self.floor_stop)
        if not (self.rim_z_min <= brim <= self.rim_z_max):
            log.error(f'[Pick] GUARD: rim_z={brim:.3f} implausible — abort')
            return 'failed'
        if math.hypot(gx - bcx, gy - bcy) > max(OPEN_X, OPEN_Y) / 2:
            log.error('[Pick] GUARD: grasp xy outside opening — abort')
            return 'failed'

        try:
            self._add_basket(bcx, bcy, brim, byaw)

            log.info('[Pick] pre-grasp (above, walls active)')
            if not self._move(pre, yaw):
                return 'failed'
            log.info('[Pick] open gripper')
            self._gripper(self.open_pos)

            # step down inside the opening
            z_top, z_bot = pre[2], gz
            n = max(1, int(math.ceil((z_top - z_bot) / self.step)))
            reached = z_top
            for i in range(1, n + 1):
                z = z_top - (z_top - z_bot) * i / n
                log.info(f'[Pick] step down -> z={z:.3f}')
                if not self._move((gx, gy, z), yaw):
                    log.warn(f'[Pick] stopped at z={reached:.3f}, grasp here')
                    break
                reached = z

            log.info('[Pick] close gripper')
            self._gripper(self.close_pos)
            log.info('[Pick] lift')
            self._move((gx, gy, brim + 0.35), yaw)
            return 'succeeded'
        finally:
            self._remove_basket()