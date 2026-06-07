#!/usr/bin/env python3
"""
CLI to test TIAGo arm manipulation — joint control + OMPL grasp with obstacle avoidance.

Controls:
  1-9  select target object from /object_centroids
  i    go to init (pregrasp hook) pose
  g    full grab sequence: init → approach → close gripper → retreat
  x    approach only (no gripper)
  a    auto-align: move arm_1 so gripper Y matches target Y
  l/r  arm_1_joint left/right (Y align)
  u/d  torso up/down
  +/-  step size
  p    print status + error to target
  o    list objects
  h    help
  q    quit
"""

import sys
import math
import termios
import tty
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from lasr_vision_interfaces.msg import Detection3DArray
from geometry_msgs.msg import Pose, Point, PointStamped, Quaternion, Twist
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
from visualization_msgs.msg import Marker, MarkerArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from pymoveit2 import MoveIt2, MoveIt2State
from rclpy.callback_groups import ReentrantCallbackGroup
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from rclpy.time import Time as ROS2Time
import rclpy.duration

ARM_JOINTS = [
    "torso_lift_joint",
    "arm_1_joint",
    "arm_2_joint",
    "arm_3_joint",
    "arm_4_joint",
    "arm_5_joint",
    "arm_6_joint",
    "arm_7_joint",
]
GRIPPER_JOINTS = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
GRIPPER_OPEN = [0.044, 0.044]
GRIPPER_CLOSE = [0.010, 0.010]

# init joint config — torso + arm_1..arm_7
# From Lab-RoCoCo-Sapienza/empower back_init() — degrees [42, 16, -109, 105, -60, -56, -108]
# converted to radians. Better grasping-ready posture than the manipulation_challenge
# pregrasp hook: elbow up, wrist pre-rotated, gripper already roughly horizontal.
#
# Two mirror configs: right elbow grabs targets on the right of base
# (target.y < 0), left elbow grabs targets on the left (target.y > 0).
# Mirroring flips arm_1/3/5/7 (the "yaw-like" joints) — arm_2/4/6 stay the same.
INIT_JOINTS_RIGHT = [
    0.35,  # torso_lift_joint
    42 * math.pi / 180,  # arm_1_joint  =  0.7330  (shoulder yaw → right)
    16 * math.pi / 180,  # arm_2_joint  =  0.2793
    -109 * math.pi / 180,  # arm_3_joint  = -1.9024
    105 * math.pi / 180,  # arm_4_joint  =  1.8326
    -60 * math.pi / 180,  # arm_5_joint  = -1.0472
    -56 * math.pi / 180,  # arm_6_joint  = -0.9773
    -108 * math.pi / 180,  # arm_7_joint  = -1.8849
]
INIT_JOINTS_LEFT = [
    0.35,  # torso_lift_joint
    -42 * math.pi / 180,  # arm_1_joint  flipped (shoulder yaw → left)
    16 * math.pi / 180,  # arm_2_joint  same
    109 * math.pi / 180,  # arm_3_joint  flipped
    105 * math.pi / 180,  # arm_4_joint  same
    60 * math.pi / 180,  # arm_5_joint  flipped
    -56 * math.pi / 180,  # arm_6_joint  same
    108 * math.pi / 180,  # arm_7_joint  flipped
]
# Backwards-compat alias (some code paths still reference INIT_JOINTS)
INIT_JOINTS = INIT_JOINTS_RIGHT

# Obstacle box size (metres) for non-target detections in the planning scene
OBSTACLE_SIZE = 0.12

SEP = "─" * 52
SEP2 = "═" * 52


class ArmCLI(Node):
    def __init__(self):
        super().__init__("arm_cli")
        self._joint_positions = {}
        self._step = 0.1
        self._centroids = []
        self._target = None

        self._cb_group = ReentrantCallbackGroup()
        self._moveit = MoveIt2(
            node=self,
            joint_names=ARM_JOINTS,
            base_link_name="base_footprint",
            end_effector_name="arm_tool_link",
            group_name="arm_torso",
            callback_group=self._cb_group,
        )
        self._moveit.planner_id = "RRTConnectkConfigDefault"
        self._moveit.max_velocity = 0.3
        self._moveit.max_acceleration = 0.3

        self._collision_pub = self.create_publisher(
            CollisionObject, "/collision_object", 10
        )

        # Mobile base velocity command — used to drive forward during grasp.
        # /cmd_vel is the topic accepted by the TIAGo Gazebo bringup;
        # /mobile_base_controller/cmd_vel does not exist in this setup.
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Diagnostic markers — publish gripper position and target position as
        # spheres in map frame so the user can visually verify alignment in
        # RViz without prospective ambiguity.
        self._debug_marker_pub = self.create_publisher(
            MarkerArray, "/arm_cli_debug", 10
        )

        self._gripper_client = ActionClient(
            self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory"
        )

        self._tf_buffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=30.0)
        )
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.create_subscription(
            Detection3DArray, "/object_centroids", self._centroids_cb, 10
        )

        self._obstacle_ids = []

    # ── callbacks ──────────────────────────────────────────────────────────

    def _js_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self._joint_positions[name] = pos

    def _centroids_cb(self, msg):
        self._centroids = msg.detections

    # ── TF helpers ─────────────────────────────────────────────────────────

    def get_joints(self):
        return [self._joint_positions.get(j, 0.0) for j in ARM_JOINTS]

    def get_ee(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                "base_footprint",
                "gripper_grasping_frame",
                ROS2Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            return tf.transform.translation
        except Exception:
            return None

    def get_ee_pose_full(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                "base_footprint",
                "gripper_grasping_frame",
                ROS2Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            return tf.transform
        except Exception as e:
            self.get_logger().warn(f"EE TF failed: {e}")
            return None

    def get_target_in_base(self):
        """Target in base_footprint via live map→base_footprint TF."""
        if self._target is None:
            return None
        try:
            tf = self._tf_buffer.lookup_transform(
                "base_footprint",
                "map",
                ROS2Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            ps = PointStamped()
            ps.header.frame_id = "map"
            # Use the TF's own stamp so do_transform_point's internal
            # consistency check doesn't try to re-lookup at stamp=0.
            ps.header.stamp = tf.header.stamp
            ps.point = self._target.point
            return do_transform_point(ps, tf).point
        except Exception as e:
            self.get_logger().warn(f"TF map→base_footprint failed: {e}")
            return None

    def _orientation_facing(self, from_x, from_y, from_z, to_x, to_y, to_z):
        """
        Quaternion that orients the EE so its X axis points from (from) to (to).
        For TIAGo, arm_tool_link X points out of the gripper, so this makes the
        gripper face the target.

        Built as a rotation matrix [x_axis | y_axis | z_axis] converted to quat.
        """
        import math

        dx = to_x - from_x
        dy = to_y - from_y
        dz = to_z - from_z
        norm = math.sqrt(dx * dx + dy * dy + dz * dz)
        if norm < 1e-6:
            return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # X axis: direction from EE to target
        xx, xy, xz = dx / norm, dy / norm, dz / norm

        # Z axis: pick world-up projected orthogonal to X (gripper opens vertically)
        # world_up = (0, 0, 1)
        # z = world_up - (world_up . x) * x ; then normalise
        dot = xz
        zx = -dot * xx
        zy = -dot * xy
        zz = 1.0 - dot * xz
        n = math.sqrt(zx * zx + zy * zy + zz * zz)
        if n < 1e-6:
            # X is aligned with world up — fall back to world Y as helper
            zx, zy, zz = 0.0, 1.0, 0.0
        else:
            zx, zy, zz = zx / n, zy / n, zz / n

        # Y axis: Z × X
        yx = zy * xz - zz * xy
        yy = zz * xx - zx * xz
        yz = zx * xy - zy * xx

        # Build rotation matrix [x y z] and convert to quaternion
        # Using standard matrix-to-quaternion conversion
        m00, m01, m02 = xx, yx, zx
        m10, m11, m12 = xy, yy, zy
        m20, m21, m22 = xz, yz, zz

        tr = m00 + m11 + m22
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (m21 - m12) / S
            qy = (m02 - m20) / S
            qz = (m10 - m01) / S
        elif m00 > m11 and m00 > m22:
            S = math.sqrt(1.0 + m00 - m11 - m22) * 2
            qw = (m21 - m12) / S
            qx = 0.25 * S
            qy = (m01 + m10) / S
            qz = (m02 + m20) / S
        elif m11 > m22:
            S = math.sqrt(1.0 + m11 - m00 - m22) * 2
            qw = (m02 - m20) / S
            qx = (m01 + m10) / S
            qy = 0.25 * S
            qz = (m12 + m21) / S
        else:
            S = math.sqrt(1.0 + m22 - m00 - m11) * 2
            qw = (m10 - m01) / S
            qx = (m02 + m20) / S
            qy = (m12 + m21) / S
            qz = 0.25 * S

        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def _point_in_base(self, map_point):
        """Transform a Point from map to base_footprint."""
        try:
            tf = self._tf_buffer.lookup_transform(
                "base_footprint",
                "map",
                ROS2Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            ps = PointStamped()
            ps.header.frame_id = "map"
            ps.header.stamp = tf.header.stamp
            ps.point = map_point
            return do_transform_point(ps, tf).point
        except Exception:
            return None

    # ── planning scene obstacles ───────────────────────────────────────────

    def _clear_obstacles(self):
        """Remove all previously added obstacle collision objects."""
        for oid in self._obstacle_ids:
            co = CollisionObject()
            co.id = oid
            co.header.frame_id = "base_footprint"
            co.operation = CollisionObject.REMOVE
            self._collision_pub.publish(co)
        self._obstacle_ids = []

    def _add_obstacles(self):
        """
        Add all detected objects except the current target as box collision
        objects in the MoveIt planning scene (frame: base_footprint).
        Detections within 10 cm of the target are also skipped (duplicate /
        overlapping detections of the same object).
        """
        self._clear_obstacles()
        if not self._centroids:
            return

        target_pt = None
        if self._target is not None:
            target_pt = self._point_in_base(self._target.point)

        added = 0
        for det in self._centroids:
            if self._target is not None and det.name == self._target.name:
                continue
            pt = self._point_in_base(det.point)
            if pt is None:
                continue
            if target_pt is not None:
                dx = pt.x - target_pt.x
                dy = pt.y - target_pt.y
                dz = pt.z - target_pt.z
                if (dx * dx + dy * dy + dz * dz) ** 0.5 < 0.10:
                    print(f"  · skip {det.name} (overlaps target)")
                    continue

            oid = f"obstacle_{det.name}_{added}"
            co = CollisionObject()
            co.id = oid
            co.header.frame_id = "base_footprint"
            co.operation = CollisionObject.ADD

            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [OBSTACLE_SIZE, OBSTACLE_SIZE, OBSTACLE_SIZE]
            co.primitives = [prim]

            pose = Pose()
            pose.position.x = pt.x
            pose.position.y = pt.y
            pose.position.z = pt.z
            pose.orientation.w = 1.0
            co.primitive_poses = [pose]

            self._collision_pub.publish(co)
            self._obstacle_ids.append(oid)
            added += 1

        if added:
            print(f"  + {added} obstacle(s) added to planning scene")
            time.sleep(0.3)  # let move_group ingest the scene update

    # ── motion primitives ──────────────────────────────────────────────────

    def _spin_wait(self, timeout=15.0):
        """Spin until motion completes. Returns True on success, False on failure/timeout.

        MoveIt2State only has IDLE / REQUESTING / EXECUTING.
        A planning failure never enters EXECUTING — it goes straight back to IDLE.
        So we wait up to 2s for the state to leave IDLE (motion accepted); if it
        never does, the plan failed. Then we wait for IDLE again (motion done).
        """
        deadline = time.time() + timeout

        # Phase 1: wait for motion to be accepted (leave IDLE)
        accepted_deadline = time.time() + 2.0
        while time.time() < accepted_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._moveit.query_state() != MoveIt2State.IDLE:
                break
        else:
            # Never left IDLE → planning failed immediately
            return False

        if self._moveit.query_state() == MoveIt2State.IDLE:
            # Returned to IDLE before we checked — planning failed
            return False

        # Phase 2: wait for motion to finish (back to IDLE)
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._moveit.query_state() == MoveIt2State.IDLE:
                return True
        return False

    def move_joints_blocking(self, joints, timeout=15.0):
        self._moveit.move_to_configuration(
            joint_positions=joints, joint_names=ARM_JOINTS
        )
        return self._spin_wait(timeout)

    def move_pose_blocking(self, pose: Pose, timeout=20.0):
        self._moveit.move_to_pose(pose=pose)
        return self._spin_wait(timeout)

    def set_gripper(self, positions, timeout=5.0):
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = GRIPPER_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.time_from_start = Duration(sec=2)
        traj.points = [pt]
        goal.trajectory = traj

        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            print("  ✗ Gripper controller not available")
            return False
        future = self._gripper_client.send_goal_async(goal)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            print("  ✗ Gripper send timeout")
            return False
        handle = future.result()
        if handle is None or not handle.accepted:
            print("  ✗ Gripper goal rejected")
            return False
        result_future = handle.get_result_async()
        deadline = time.time() + timeout
        while not result_future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    # ── high-level moves ───────────────────────────────────────────────────

    def pick_init_config(self):
        """
        Return INIT_JOINTS_LEFT or INIT_JOINTS_RIGHT depending on target.y in
        base_footprint. Right elbow for target on the right (y < 0), left
        for left. Defaults to RIGHT if no target.
        """
        if self._target is None:
            return INIT_JOINTS_RIGHT, "RIGHT (default)"
        t = self.get_target_in_base()
        if t is None:
            return INIT_JOINTS_RIGHT, "RIGHT (no TF)"
        if t.y > 0.05:
            return INIT_JOINTS_LEFT, "LEFT"
        return INIT_JOINTS_RIGHT, "RIGHT"

    def go_init(self):
        """Move to init (pregrasp hook) joint configuration, side chosen by target."""
        config, side = self.pick_init_config()
        print(f"\n  → Moving to init pose [{side}]...")
        ok = self.move_joints_blocking(config, timeout=20.0)
        print(f'  {"✓ Init pose reached" if ok else "✗ Init pose timeout"}')
        self.print_status()

    def drive_base(self, linear_x: float, duration: float, rate_hz: float = 20.0):
        """Publish constant Twist to /mobile_base_controller/cmd_vel for `duration` s."""
        twist = Twist()
        twist.linear.x = linear_x
        period = 1.0 / rate_hz
        end = time.time() + duration
        while time.time() < end:
            self._cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(period)
        # Stop
        stop = Twist()
        for _ in range(5):
            self._cmd_vel_pub.publish(stop)
            time.sleep(0.05)

    def get_ee_in_map(self):
        """EE position in map frame (live, follows AMCL corrections)."""
        try:
            tf = self._tf_buffer.lookup_transform(
                "map",
                "gripper_grasping_frame",
                ROS2Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            return tf.transform.translation
        except Exception:
            return None

    def get_ee_in_odom(self):
        """EE position in odom frame (continuous, AMCL-corrections free)."""
        try:
            tf = self._tf_buffer.lookup_transform(
                "odom",
                "gripper_grasping_frame",
                ROS2Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            return tf.transform.translation
        except Exception:
            return None

    def map_point_to_odom(self, map_x: float, map_y: float, map_z: float = 0.0):
        """Snapshot a map-frame point into odom once. Use the resulting odom
        coordinates for closed-loop drive control — odom is smooth and
        unaffected by AMCL re-localisations that would otherwise jump the
        target mid-motion."""
        try:
            tf = self._tf_buffer.lookup_transform(
                "odom", "map", ROS2Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )
            ps = PointStamped()
            ps.header.frame_id = "map"
            ps.header.stamp = tf.header.stamp
            ps.point.x = map_x
            ps.point.y = map_y
            ps.point.z = map_z
            p = do_transform_point(ps, tf).point
            return p.x, p.y, p.z
        except Exception as e:
            self.get_logger().warn(f"map→odom failed: {e}")
            return None

    def rotate_base_toward(
        self,
        target_map_x: float,
        target_map_y: float,
        tolerance_rad: float = 0.03,
        max_omega: float = 0.08,
        timeout: float = 25.0,
    ):
        """
        Rotate base in place so the GRIPPER's straight-line trajectory (which
        equals the base heading direction, since rigid transform) passes
        through the target. We compute desired yaw from the gripper→target
        vector, not the base→target vector — the gripper is laterally offset
        from the base, so aiming the base at the target makes the gripper
        miss sideways.
        """
        import math

        twist = Twist()
        end = time.time() + timeout
        last_print = 0.0
        while time.time() < end:
            try:
                tf = self._tf_buffer.lookup_transform(
                    "map",
                    "base_footprint",
                    ROS2Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
            except Exception:
                self._cmd_vel_pub.publish(Twist())
                return False
            ee_map = self.get_ee_in_map()
            if ee_map is None:
                self._cmd_vel_pub.publish(Twist())
                return False
            q = tf.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)
            # Desired heading = direction from gripper to target (in map).
            # When the base advances in +X, the gripper translates in this
            # same direction (rigid body), so it heads straight at the target.
            desired = math.atan2(target_map_y - ee_map.y, target_map_x - ee_map.x)
            err = math.atan2(math.sin(desired - yaw), math.cos(desired - yaw))
            if abs(err) < tolerance_rad:
                break
            omega = max(min(err * 0.8, max_omega), -max_omega)
            twist.angular.z = omega
            self._cmd_vel_pub.publish(twist)
            if time.time() - last_print > 0.5:
                print(
                    f"    yaw={yaw:+.3f} desired={desired:+.3f} err={err:+.3f} ω={omega:+.2f}"
                )
                last_print = time.time()
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.05)
        for _ in range(10):
            self._cmd_vel_pub.publish(Twist())
            time.sleep(0.02)
        return True

    def drive_base_until_target(
        self,
        target_map_x: float,
        target_map_y: float,
        stop_offset: float = 0.12,
        linear_x: float = 0.08,
        tolerance: float = 0.04,
        timeout: float = 20.0,
        align_period: float = 1.0,
        align_tolerance: float = 0.02,
    ):
        """
        Drive base forward until EE reaches target (everything in MAP frame).
        Continuously re-aligns arm_1 every `align_period` seconds so the
        gripper stays Y-locked on the target during motion — compensates for
        small base drift and refined target estimates. Twist publishing is not
        blocked by the arm command since they go to independent controllers.
        """
        twist = Twist()
        twist.linear.x = linear_x
        end = time.time() + timeout
        last_print = 0.0
        last_align = 0.0
        reached = False
        min_dist = float("inf")
        increasing_count = 0
        while time.time() < end:
            ee_map = self.get_ee_in_map()
            if ee_map is None:
                self._cmd_vel_pub.publish(Twist())
                return False
            # Stop point = target shifted back by stop_offset along the EE→target vector.
            ddx = target_map_x - ee_map.x
            ddy = target_map_y - ee_map.y
            d_full = (ddx * ddx + ddy * ddy) ** 0.5
            if d_full > 1e-3:
                goal_x = target_map_x - (ddx / d_full) * stop_offset
                goal_y = target_map_y - (ddy / d_full) * stop_offset
            else:
                goal_x = target_map_x
                goal_y = target_map_y
            err_x = goal_x - ee_map.x
            err_y = goal_y - ee_map.y
            dist = (err_x * err_x + err_y * err_y) ** 0.5
            if dist < tolerance:
                reached = True
                break
            if dist < min_dist:
                min_dist = dist
                increasing_count = 0
            elif dist > min_dist + 0.02:
                increasing_count += 1
                if increasing_count >= 5:
                    print(f"    ⚠ closest approach reached: min_dist={min_dist:.3f}")
                    reached = True
                    break
            self._cmd_vel_pub.publish(twist)

            # Periodic Y re-alignment of the gripper via arm_1. We don't stop
            # the base — twist keeps being published from the next loop tick.
            if time.time() - last_align > align_period:
                ee_bf = self.get_ee()
                t_bf = self.get_target_in_base()
                if ee_bf is not None and t_bf is not None:
                    dy = t_bf.y - ee_bf.y
                    if abs(dy) > align_tolerance:
                        joints = self.get_joints()
                        new_arm1 = max(-1.1, min(1.5, joints[1] + 2.0 * dy))
                        if abs(new_arm1 - joints[1]) > 1e-3:
                            joints[1] = new_arm1
                            self._moveit.move_to_configuration(
                                joint_positions=joints, joint_names=ARM_JOINTS
                            )
                            print(
                                f"    ⟲ realign: dy={dy:+.3f} → arm_1={new_arm1:+.3f}"
                            )
                last_align = time.time()

            if time.time() - last_print > 0.5:
                print(
                    f"    EE(map): ({ee_map.x:.3f},{ee_map.y:.3f}) → goal_map({goal_x:.3f},{goal_y:.3f})  dist={dist:.3f}"
                )
                last_print = time.time()
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.05)
        # Always stop
        for _ in range(10):
            self._cmd_vel_pub.publish(Twist())
            time.sleep(0.02)
        return reached

    def approach(self):
        """
        Approach strategy: pick the LEFT/RIGHT init config based on target.y,
        align Z via torso, then drive the base forward. NO arm_1 sweep — the
        mirrored init already places the gripper on the correct side of the
        body for the target, and the base drives straight ahead because the
        user has already positioned TIAGo facing the object.
        """
        if self._target is None:
            print("  ✗ No target selected")
            return False
        t = self.get_target_in_base()
        if t is None:
            print("  ✗ TF unavailable")
            return False
        ee_tf = self.get_ee_pose_full()
        if ee_tf is None:
            print("  ✗ EE unavailable")
            return False

        GRASP_Z_OFFSET = -0.05
        target_z = t.z + GRASP_Z_OFFSET

        print(
            f"\n  EE now    : x={ee_tf.translation.x:.3f}  y={ee_tf.translation.y:.3f}  z={ee_tf.translation.z:.3f}"
        )
        print(f"  Target(bf): x={t.x:.3f}  y={t.y:.3f}  z={target_z:.3f}")

        # ── Stage 1: align Z using torso ──────────────────────────────────
        print(f"  → Stage 1: align Z via torso")
        ee = self.get_ee()
        if ee is not None:
            dz = target_z - ee.z
            joints = self.get_joints()
            new_torso = max(0.0, min(0.35, joints[0] + dz))
            print(f"    dz={dz:+.3f} → torso {joints[0]:.3f} → {new_torso:.3f}")
            joints[0] = new_torso
            self.move_joints_blocking(joints, timeout=8.0)

        # ── Stage 1.5: align gripper Y to target via arm_1 ────────────────
        # Critical: do this BEFORE driving forward. Without a good initial
        # Y-alignment, the base passes the target in X before the in-loop
        # realign converges, and drive_base_until_target gives up at
        # "closest approach reached" with several cm of Y error.
        print(f"  → Stage 1.5: align gripper Y via arm_1")
        self.align_arm1_to_target(tolerance=0.01, max_iters=5)

        # ── Stage 2: drive base forward (everything in MAP) ───────────────
        target_map_x = self._target.point.x
        target_map_y = self._target.point.y

        import math

        ee_map = self.get_ee_in_map()
        if ee_map is None:
            print("  ✗ EE in map unavailable")
            return False
        ddx = target_map_x - ee_map.x
        ddy = target_map_y - ee_map.y
        d = math.sqrt(ddx * ddx + ddy * ddy)
        stop_offset = 0.12

        print(f"  → Stage 2: drive base (map frame)")
        print(f"            Target (map) : ({target_map_x:.3f}, {target_map_y:.3f})")
        print(f"            EE (map)     : ({ee_map.x:.3f}, {ee_map.y:.3f})")
        print(f"            Distance     : {d - stop_offset:.3f} m")

        if d - stop_offset < 0.02:
            print("  ✓ Already at grasp distance")
            return True

        ok = self.drive_base_until_target(
            target_map_x=target_map_x,
            target_map_y=target_map_y,
            stop_offset=stop_offset,
            linear_x=0.08,
            tolerance=0.04,
            timeout=max(15.0, d / 0.08 * 2.0),
        )
        if not ok:
            print("  ⚠ Base did not reach target within tolerance")
        else:
            print("  ✓ Base advanced to grasp distance")
        return True

    def grab(self):
        """
        Full grab sequence:
          1. go to init pose
          2. open gripper
          3. approach (wp1: Y+Z, wp2: X) — obstacles in scene
          4. close gripper
          5. retreat to init pose
          6. clear obstacles
        """
        if self._target is None:
            print("  ✗ No target selected")
            return

        print(f"\n{SEP2}")
        print(f"  GRAB: {self._target.name}")
        print(f"{SEP2}")

        print("\n  [1/5] Init pose")
        self.go_init()

        print("\n  [2/5] Open gripper")
        self.set_gripper(GRIPPER_OPEN)

        print("\n  [3/5] Approach")
        ok = self.approach()
        if not ok:
            print("  ✗ Approach failed — aborting")
            self._clear_obstacles()
            return

        # Plunge: drop the torso 8 cm AND push the base forward 40 cm so the
        # gripper closes around the object instead of just grazing it.
        print("\n  [4a] Plunge: torso down 8 cm")
        joints = self.get_joints()
        new_torso = max(0.0, joints[0] - 0.08)
        print(f"      torso {joints[0]:.3f} → {new_torso:.3f}")
        joints[0] = new_torso
        self.move_joints_blocking(joints, timeout=6.0)

        print("\n  [4b] Plunge: drive base forward 40 cm")
        plunge_distance = 0.40
        self.drive_base(linear_x=0.05, duration=plunge_distance / 0.05)
        print(f"  ✓ Plunged {plunge_distance:.2f} m forward")

        print("\n  [4/6] Close gripper")
        self.set_gripper(GRIPPER_CLOSE)
        time.sleep(0.5)

        # Retreat base BACKWARDS by the same distance we drove in. The arm is
        # still extended toward the (now-grasped) object — by reversing the
        # base we pull the gripper safely away from the table without arm planning.
        print("\n  [5/6] Drive base backwards")
        retreat_distance = 0.20  # back off 20 cm
        self.drive_base(linear_x=-0.05, duration=retreat_distance / 0.05)
        print(f"  ✓ Retreated {retreat_distance:.2f} m")

        print("\n  [6/6] Retreat arm to init pose (joint-space)")
        config, side = self.pick_init_config()
        print(f"      using {side} elbow config")
        ok = self.move_joints_blocking(config, timeout=20.0)
        print(f'  {"✓ Init reached" if ok else "✗ Init timeout"}')

        self._clear_obstacles()

        print(f"\n{SEP2}")
        print(f"  Grab sequence complete")
        print(f"{SEP2}\n")
        self.print_status()

    def move_arm1(self, delta):
        joints = self.get_joints()
        joints[1] += delta
        self.move_joints_blocking(joints)
        self.print_status()

    def align_arm1_to_target(self, tolerance: float = 0.01, max_iters: int = 5):
        """
        Iteratively move arm_1_joint so the gripper's Y in base_footprint
        matches the target's Y. The dEE.y / d(arm_1) ratio is non-linear
        across the workspace (~0.4-0.55), so a single closed-form correction
        leaves residual error. We re-measure after each move and re-correct
        until |dy| < tolerance (default 1 cm) or max_iters is exhausted.
        """
        if self._target is None:
            print("  ✗ No target selected")
            return
        for it in range(max_iters):
            for _ in range(3):
                rclpy.spin_once(self, timeout_sec=0.05)
            ee = self.get_ee()
            t = self.get_target_in_base()
            if ee is None or t is None:
                print("  ✗ TF unavailable")
                return
            dy = t.y - ee.y
            if abs(dy) < tolerance:
                print(f"  ✓ aligned after {it} iter(s) — dy={dy:+.4f} m")
                self.print_status()
                return
            delta = 2.0 * dy
            joints = self.get_joints()
            new_arm1 = max(-1.1, min(1.5, joints[1] + delta))
            actual_delta = new_arm1 - joints[1]
            print(
                f"  [iter {it+1}] dy={dy:+.3f}  Δarm_1={actual_delta:+.3f}  "
                f"({joints[1]:+.3f} → {new_arm1:+.3f})"
            )
            if abs(actual_delta) < 1e-3:
                print("  ✗ at joint limit, cannot reduce further")
                self.print_status()
                return
            joints[1] = new_arm1
            ok = self.move_joints_blocking(joints, timeout=8.0)
            if not ok:
                print("  ✗ motion failed")
                return
        print(f"  ⚠ max_iters reached")
        self.print_status()

    def move_torso(self, delta):
        joints = self.get_joints()
        joints[0] = max(0.0, min(0.35, joints[0] + delta))
        self.move_joints_blocking(joints)
        self.print_status()

    # ── display ────────────────────────────────────────────────────────────

    def print_objects(self):
        print(f"\n{SEP2}")
        if not self._centroids:
            print("  No detections on /object_centroids")
            print(
                f"  Run: ros2 topic pub --once /detect std_msgs/msg/String \"data: 'bottle'\""
            )
        else:
            print(f'  {"#":<3} {"Name":<20} {"x":>6} {"y":>6} {"z":>6}')
            print(f"  {SEP}")
            for i, det in enumerate(self._centroids):
                marker = (
                    " <-- TARGET"
                    if self._target and det.name == self._target.name
                    else ""
                )
                print(
                    f"  {i+1:<3} {det.name:<20} {det.point.x:>6.3f} {det.point.y:>6.3f} {det.point.z:>6.3f}{marker}"
                )
        print(f"{SEP2}\n")

    def _publish_debug_markers(self, ee_map):
        """
        Publish three RViz markers in /arm_cli_debug (frame: map):
          - yellow sphere at the gripper (EE in map)
          - magenta sphere at the target (from _target.point)
          - green line connecting them
        This gives an unambiguous visual check of alignment, free of the
        prospective distortion that top-down RViz views can introduce.
        """
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        if ee_map is not None:
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = stamp
            m.ns = "arm_cli"
            m.id = 0
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = ee_map.x
            m.pose.position.y = ee_map.y
            m.pose.position.z = ee_map.z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.06
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 0.9
            ma.markers.append(m)

        if self._target is not None:
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = stamp
            m.ns = "arm_cli"
            m.id = 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = self._target.point
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.06
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 1.0, 0.9
            ma.markers.append(m)

            if ee_map is not None:
                line = Marker()
                line.header.frame_id = "map"
                line.header.stamp = stamp
                line.ns = "arm_cli"
                line.id = 2
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.pose.orientation.w = 1.0
                line.scale.x = 0.015
                line.color.r, line.color.g, line.color.b, line.color.a = (
                    0.0,
                    1.0,
                    0.0,
                    0.9,
                )
                p1 = Point(x=ee_map.x, y=ee_map.y, z=ee_map.z)
                p2 = Point(
                    x=self._target.point.x,
                    y=self._target.point.y,
                    z=self._target.point.z,
                )
                line.points = [p1, p2]
                ma.markers.append(line)

        if ma.markers:
            self._debug_marker_pub.publish(ma)

    def print_status(self):
        ee = self.get_ee()
        ee_map = self.get_ee_in_map()
        joints = self.get_joints()
        self._publish_debug_markers(ee_map)

        print(f"\n{SEP}")
        print(f"  JOINTS")
        print(f"    torso  : {joints[0]:>+7.3f} m")
        print(
            f"    arm_1  : {joints[1]:>+7.3f} rad   arm_2: {joints[2]:>+7.3f}   arm_3: {joints[3]:>+7.3f}"
        )
        print(
            f"    arm_4  : {joints[4]:>+7.3f} rad   arm_5: {joints[5]:>+7.3f}   arm_6: {joints[6]:>+7.3f}   arm_7: {joints[7]:>+7.3f}"
        )
        print(f"  EE (gripper / arm_tool_link)")
        if ee:
            print(f"    base_fp:  x={ee.x:>+.3f}  y={ee.y:>+.3f}  z={ee.z:>+.3f}")
        else:
            print(f"    base_fp:  [TF unavailable]")
        if ee_map:
            print(
                f"    map    :  x={ee_map.x:>+.3f}  y={ee_map.y:>+.3f}  z={ee_map.z:>+.3f}"
            )

        if self._target and ee:
            print(f"  {SEP}")
            print(f"  TARGET: {self._target.name}")
            print(
                f"    map    :  x={self._target.point.x:>+.3f}  y={self._target.point.y:>+.3f}  z={self._target.point.z:>+.3f}"
            )
            t = self.get_target_in_base()
            if t is None:
                print(f"    base_fp:  [TF unavailable]")
            else:
                dx = t.x - ee.x
                dy = t.y - ee.y
                dz = t.z - ee.z
                dist_xy = math.sqrt(dx**2 + dy**2)
                dist_3d = math.sqrt(dx**2 + dy**2 + dz**2)
                print(f"    base_fp:  x={t.x:>+.3f}  y={t.y:>+.3f}  z={t.z:>+.3f}")
                print(f"  EE→TARGET error (in base_footprint)")
                print(
                    f"    dx={dx:>+.3f}  dy={dy:>+.3f}  dz={dz:>+.3f}   |xy|={dist_xy:.3f}m  |3d|={dist_3d:.3f}m"
                )
                x_ok = "✓" if abs(dx) < 0.05 else "✗"
                y_ok = "✓" if abs(dy) < 0.05 else "✗"
                z_ok = "✓" if abs(dz) < 0.05 else "✗"
                print(f"    X {x_ok}     Y {y_ok}     Z {z_ok}    (✓ = within 5cm)")
                if abs(dy) >= 0.05:
                    print(
                        f'    hint:    press {"l" if dy > 0 else "r"} to swing arm_1 toward target Y'
                    )
                if abs(dz) >= 0.05:
                    print(
                        f'    hint:    press {"u" if dz > 0 else "d"} to move torso toward target Z'
                    )
            if ee_map:
                ddx = self._target.point.x - ee_map.x
                ddy = self._target.point.y - ee_map.y
                ddz = self._target.point.z - ee_map.z
                d_xy = math.sqrt(ddx**2 + ddy**2)
                print(f"  EE→TARGET error (in map)")
                print(
                    f"    dx={ddx:>+.3f}  dy={ddy:>+.3f}  dz={ddz:>+.3f}   |xy|={d_xy:.3f}m"
                )
        elif self._target:
            print(f"  TARGET: {self._target.name} (EE unavailable)")
        else:
            print(f"  No target selected — press 1-9 to select")
        print(f"  Step: {self._step:.2f} rad")
        print(f"{SEP}\n")


# ── terminal I/O ───────────────────────────────────────────────────────────


def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def print_help():
    print(f"\n  Controls:")
    print(f"  1-9  select target object")
    print(f"  i    go to init (pregrasp) pose")
    print(f"  g    full grab: init → approach → close → retreat")
    print(f"  x    approach only (no gripper, obstacles in scene)")
    print(f"  a    auto-align: move arm_1 so gripper Y matches target Y")
    print(f"  l/r  arm_1_joint left/right (Y align)")
    print(f"  u/d  torso up/down")
    print(f"  +/-  step size")
    print(f"  p    print status")
    print(f"  o    list objects")
    print(f"  h    help")
    print(f"  q    quit\n")


def main():
    rclpy.init()
    node = ArmCLI()

    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node._joint_positions:
            break

    print_help()
    node.print_objects()
    node.print_status()

    while True:
        ch = getch()

        if ch == "q":
            break
        elif ch in "123456789":
            idx = int(ch) - 1
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.05)
            if idx < len(node._centroids):
                node._target = node._centroids[idx]
                # Target lives in MAP. All control loops resolve map→base
                # live, so AMCL corrections are followed automatically.
                print(
                    f"\n  → Target: {node._target.name} @ map({node._target.point.x:.3f}, {node._target.point.y:.3f}, {node._target.point.z:.3f})"
                )
                node.print_status()
            else:
                print(
                    f"\n  ✗ No object at index {idx+1} (only {len(node._centroids)} detected)"
                )
                node.print_objects()
        elif ch == "i":
            node.go_init()
        elif ch == "g":
            node.grab()
        elif ch == "x":
            node.approach()
            node.print_status()
        elif ch == "a":
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.05)
            node.align_arm1_to_target()
        elif ch == "l":
            node.move_arm1(+node._step)
        elif ch == "r":
            node.move_arm1(-node._step)
        elif ch == "u":
            node.move_torso(+node._step)
        elif ch == "d":
            node.move_torso(-node._step)
        elif ch == "+":
            node._step = min(node._step + 0.05, 0.5)
            print(f"  Step → {node._step:.2f} rad")
        elif ch == "-":
            node._step = max(node._step - 0.02, 0.02)
            print(f"  Step → {node._step:.2f} rad")
        elif ch == "p":
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.05)
            node.print_status()
        elif ch == "o":
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.05)
            node.print_objects()
        elif ch == "h":
            print_help()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
