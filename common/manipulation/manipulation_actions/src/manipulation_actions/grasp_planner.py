#!/usr/bin/env python3
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, PoseStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient

from pymoveit2 import MoveIt2, MoveIt2Gripper

# TIAGo arm+torso planning group
TIAGO_ARM_JOINTS = [
    'torso_lift_joint',
    'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint',
    'arm_5_joint', 'arm_6_joint', 'arm_7_joint',
]
TIAGO_GRIPPER_JOINTS = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
GRIPPER_OPEN  = [0.044, 0.044]
GRIPPER_CLOSE = [0.010, 0.010]

# Pregrasp waypoints from manipulation_challenge pregrasp_motion.yaml
PREGRASP_WAYPOINTS = [
    ([0.34, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0],  3.0),
    ([0.34, 0.10,  0.47, -0.20, 1.56, -1.58, 0.25, 0.0],  8.5),
    ([0.34, 0.10,  0.47, -0.20, 1.56,  1.60, 0.25, 1.19], 10.5),
]
HOME_JOINTS = [0.0, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0]


class GraspPlanner:
    def __init__(self, node, params: dict):
        self.node = node
        self.params = params
        self._joint_positions = {}

        callback_group = ReentrantCallbackGroup()

        self._moveit = MoveIt2(
            node=node,
            joint_names=TIAGO_ARM_JOINTS,
            base_link_name='base_footprint',
            end_effector_name='arm_tool_link',
            group_name='arm_torso',
            callback_group=callback_group,
        )
        self._moveit.planner_id = 'RRTConnectkConfigDefault'
        self._moveit.max_velocity = 0.3
        self._moveit.max_acceleration = 0.3

        self._gripper_client = ActionClient(node, FollowJointTrajectory,
                                            '/gripper_controller/follow_joint_trajectory')

        node.create_subscription(JointState, '/joint_states', self._js_callback, 10)

    def _js_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._joint_positions[name] = pos

    def _get_arm_joints(self):
        return [self._joint_positions.get(j, 0.0) for j in TIAGO_ARM_JOINTS]

    def _move_to_joints(self, positions):
        self._moveit.move_to_configuration(
            joint_positions=positions,
            joint_names=TIAGO_ARM_JOINTS,
        )
        # Wait for execution to complete
        while self._moveit.query_state() not in (
            None,
        ):
            time.sleep(0.1)

    def _set_gripper(self, positions):
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = TIAGO_GRIPPER_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(sec=2)
        traj.points = [pt]
        goal.trajectory = traj

        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error('Gripper controller not available')
            return
        future = self._gripper_client.send_goal_async(goal)
        _spin_until(self.node, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error('Gripper goal rejected')
            return
        result_future = goal_handle.get_result_async()
        _spin_until(self.node, result_future, timeout_sec=5.0)

    def _pregrasp(self):
        self.node.get_logger().info('Executing pregrasp...')
        # Prepend current robot state as start point so MoveIt2 accepts the trajectory
        current = self._get_arm_joints()
        waypoints = [([current[i] for i in range(len(TIAGO_ARM_JOINTS))], 0.5)] + list(PREGRASP_WAYPOINTS)
        self._moveit.execute(self._build_trajectory(waypoints))
        time.sleep(12.0)
        self.node.get_logger().info('Pregrasp done')

    def _build_trajectory(self, waypoints):
        traj = JointTrajectory()
        traj.joint_names = TIAGO_ARM_JOINTS
        for positions, t in waypoints:
            pt = JointTrajectoryPoint()
            pt.positions = list(positions)
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            traj.points.append(pt)
        return traj

    def _align_y(self, target_y: float):
        tolerance = self.params['align_y_tolerance']
        step_coarse = self.params['align_y_step_coarse']
        step_fine = self.params['align_y_step_fine']
        max_iter = self.params['align_max_iterations']

        for i in range(max_iter):
            joints = self._get_arm_joints()
            y_delta = target_y - joints[1] * 0.5  # rough heuristic

            if abs(y_delta) < tolerance:
                self.node.get_logger().info(f'Y aligned after {i} iterations')
                return True

            step = step_coarse if abs(y_delta) > 0.10 else step_fine
            joints[1] += (1 if y_delta > 0 else -1) * step
            self._moveit.move_to_configuration(joint_positions=joints, joint_names=TIAGO_ARM_JOINTS)
            time.sleep(2.0)
            self.node.get_logger().info(f'align_y iter {i}: y_delta={y_delta:.3f} arm1={joints[1]:.3f}')

        self.node.get_logger().warn('align_y: max iterations reached')
        return False

    def _approach_xz(self, target: Point):
        self.node.get_logger().info(f'Approaching x={target.x:.2f} z={target.z:.2f}')
        self._set_gripper(GRIPPER_OPEN)
        time.sleep(0.5)

    def _retreat(self):
        joints = self._get_arm_joints()
        joints[0] = min(joints[0] + 0.05, 0.35)
        self._moveit.move_to_configuration(joint_positions=joints, joint_names=TIAGO_ARM_JOINTS)
        time.sleep(3.0)
        self.node.get_logger().info('Retreat done')

    def go_home(self):
        self.node.get_logger().info('Going home...')
        self._moveit.move_to_configuration(joint_positions=HOME_JOINTS, joint_names=TIAGO_ARM_JOINTS)
        time.sleep(5.0)

    def init_grasp(self):
        self.node.get_logger().info('Init grasp position...')
        self._pregrasp()

    def pick(self, target: Point):
        self.node.get_logger().info(f'Pick target: ({target.x:.2f}, {target.y:.2f}, {target.z:.2f})')

        self.node.get_logger().info('Step 1: pregrasp')
        self._pregrasp()

        self.node.get_logger().info('Step 2: align Y')
        self._align_y(target.y)

        self.node.get_logger().info('Step 3: approach')
        self._approach_xz(target)

        self.node.get_logger().info('Step 4: close gripper')
        self._set_gripper(GRIPPER_CLOSE)
        time.sleep(0.5)

        self.node.get_logger().info('Step 5: retreat')
        self._retreat()

        self.node.get_logger().info('Pick sequence completed')


def _spin_until(node, future, timeout_sec=10.0):
    import rclpy
    start = time.time()
    while not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start > timeout_sec:
            node.get_logger().warn('_spin_until: timeout')
            break
