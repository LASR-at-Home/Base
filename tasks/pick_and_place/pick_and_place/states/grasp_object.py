import time
import math
from threading import Thread

import yasmin
import yasmin_ros

import rclpy
import rclpy.duration
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time as ROS2Time

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PointStamped, Pose, Quaternion, Point
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Empty

import tf2_ros
from tf2_geometry_msgs import do_transform_point

import numpy as np
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from rcl_interfaces.srv import GetParameters

from pick_and_place.states.arm_ik import ArmIK, point_forward

try:
    from pymoveit2 import MoveIt2
except ImportError:  # only the MoveIt grasp mode needs pymoveit2
    MoveIt2 = None


ARM_JOINTS = [
    "torso_lift_joint", "arm_1_joint", "arm_2_joint", "arm_3_joint",
    "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint",
]
GRIPPER_JOINTS = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
GRIPPER_OPEN = [0.044, 0.044]
GRIPPER_CLOSE = [0.010, 0.010]
DEPTH_TOPIC = "/head_front_camera/depth/image_raw"
INFO_TOPIC = "/head_front_camera/rgb/camera_info"
# Clearance above the table top (m) for each part of the arm, allowing for its
# thickness: shoulder and upper arm, elbow and forearm, wrist, then the gripper.
KEEP_UP = {
    "arm_2_joint": 0.07, "arm_3_joint": 0.07, "arm_4_joint": 0.07, "arm_5_joint": 0.07,
    "arm_6_joint": 0.04, "arm_7_joint": 0.04,
}
GRIPPER_CLEARANCE = 0.02

# Pregrasp = the team's "reach_arm" pose (skills/config/motions.yaml), arm in front of
# the robot, torso up. The old values put the arm beside/behind the robot on this TIAGo
# (arm_1 small = arm pointing back; its limits are 0.07..2.68 rad).
INIT_JOINTS_RIGHT = [0.35, 1.61, -0.93, -3.14, 1.83, -1.58, -0.53, 0.00]
INIT_JOINTS_LEFT = list(INIT_JOINTS_RIGHT)

DEFAULT_TABLE_POS = [1.3, 0.0, 0.37]
DEFAULT_TABLE_SIZE = [1.2, 1.6, 0.74]

TUCK_JOINTS = [
    0.15, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.00,
]


class GraspObject(yasmin.State):
    """
    Arm grasp planned with the robot's own arm model (IK from /robot_description):
    face the object, measure the free space with the depth camera, drive only as
    far as needed, reach in with the arm, close, check the fingers, pull out.

    Switched by pick_and_place.grasp.use_moveit:
      * False (default, SIM): joint moves go STRAIGHT to /arm_controller +
        /torso_controller (no collision check). The sim's MoveIt is broken for
        grasping (no IK; the table box blocks the path to objects on it), so direct
        execution of the fixed, table-safe poses is the only thing that works.
      * True (REAL ROBOT): the same targets are PLANNED + executed by MoveIt
        (collision-checked). If the table box blocks planning, set
        pick_and_place.grasp.publish_box: false (or shrink the box).
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("selected_object")
        self.node = yasmin_ros.logger_node
        self._ready = False
        self._use_moveit = False
        self._moveit = None
        self._mn = None
        self._joints = {}

    def _setup(self):
        if self._ready:
            return
        self._use_moveit = bool(self._param("pick_and_place.grasp.use_moveit", False))
        yasmin.YASMIN_LOG_INFO(
            f"Grasp mode: {'MoveIt (collision-checked)' if self._use_moveit else 'direct controllers'}"
        )

        self._mn = rclpy.create_node("grasp_object_moveit")

        cb = ReentrantCallbackGroup()
        if MoveIt2 is None:
            if self._use_moveit:
                yasmin.YASMIN_LOG_WARN("pymoveit2 not installed: using direct controllers instead of MoveIt")
            self._use_moveit = False
        else:
            self._moveit = MoveIt2(
                node=self._mn,
                joint_names=ARM_JOINTS,
                base_link_name="base_footprint",
                end_effector_name="arm_tool_link",
                group_name="arm_torso",
                callback_group=cb,
            )
            self._moveit.planner_id = "RRTConnectkConfigDefault"
            self._moveit.max_velocity = 0.3
            self._moveit.max_acceleration = 0.3

        self._gripper = ActionClient(
            self._mn, FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
        )
        self._arm_ctrl = ActionClient(
            self._mn, FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )
        self._torso_ctrl = ActionClient(
            self._mn, FollowJointTrajectory,
            "/torso_controller/follow_joint_trajectory",
        )
        self._cmd_vel = self._mn.create_publisher(Twist, "/cmd_vel", 10)
        self._coll_pub = self._mn.create_publisher(CollisionObject, "/collision_object", 10)
        self._clear_octo_cli = self._mn.create_client(Empty, "/clear_octomap")
        self._tf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf, self._mn)
        self._mn.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        self._mexec = MultiThreadedExecutor()
        self._mexec.add_node(self._mn)
        self._mthread = Thread(target=self._mexec.spin, daemon=True)
        self._mthread.start()

        if self._use_moveit:
            self._disable_start_tolerance()
        self._ready = True

    def _disable_start_tolerance(self):
        try:
            cli = self._mn.create_client(SetParameters, "/move_group/set_parameters")
            if not cli.wait_for_service(timeout_sec=5.0):
                yasmin.YASMIN_LOG_WARN(
                    "move_group params unavailable; set allowed_start_tolerance by hand."
                )
                return
            req = SetParameters.Request()
            p = Parameter()
            p.name = "trajectory_execution.allowed_start_tolerance"
            p.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=0.0
            )
            req.parameters = [p]
            cli.call_async(req)
            time.sleep(1.0)
            yasmin.YASMIN_LOG_INFO("move_group allowed_start_tolerance set to 0.0")
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"Could not set allowed_start_tolerance: {e}")

    def _publish_table_box(self):
        pos = list(self._param("pick_and_place.table.collision.position", DEFAULT_TABLE_POS))
        size = list(self._param("pick_and_place.table.collision.size", DEFAULT_TABLE_SIZE))
        co = CollisionObject()
        co.header.frame_id = "map"
        co.id = "table"
        co.operation = CollisionObject.ADD
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(size[0]), float(size[1]), float(size[2])]
        co.primitives.append(box)
        p = Pose()
        p.position.x, p.position.y, p.position.z = float(pos[0]), float(pos[1]), float(pos[2])
        p.orientation = self._table_orientation()
        co.primitive_poses.append(p)
        co.pose.orientation.w = 1.0
        for _ in range(3):
            self._coll_pub.publish(co)
            time.sleep(0.2)
        yasmin.YASMIN_LOG_INFO(
            f"Published table box (map) pos={pos} size={size} "
            f"quat z={p.orientation.z:.3f} w={p.orientation.w:.3f}"
        )

    def _remove_table_box(self):
        co = CollisionObject()
        co.header.frame_id = "map"
        co.id = "table"
        co.operation = CollisionObject.REMOVE
        for _ in range(3):
            self._coll_pub.publish(co)
            time.sleep(0.1)
        yasmin.YASMIN_LOG_INFO("Removed table box for navigation.")

    def _clear_octomap(self):
        try:
            if self._clear_octo_cli.wait_for_service(timeout_sec=2.0):
                self._clear_octo_cli.call_async(Empty.Request())
                time.sleep(0.4)
        except Exception:
            pass

    def _js_cb(self, msg):
        for n, p in zip(msg.name, msg.position):
            self._joints[n] = p

    def _arm_joints(self):
        return [self._joints.get(j, 0.0) for j in ARM_JOINTS]

    def _param(self, name, default):
        try:
            v = self.node.get_parameter(name).value
            return v if v is not None else default
        except Exception:
            return default

    def _quat_param(self, base):
        lst = self._param(base, None)
        if isinstance(lst, (list, tuple)) and len(lst) == 4:
            return [float(v) for v in lst]
        x = self._param(base + ".x", None)
        y = self._param(base + ".y", None)
        z = self._param(base + ".z", None)
        w = self._param(base + ".w", None)
        if None not in (z, w):
            return [float(x or 0.0), float(y or 0.0), float(z), float(w)]
        return None

    def _table_orientation(self):
        o = self._quat_param("pick_and_place.table.collision.orientation")
        if o is None:
            o = self._quat_param("pick_and_place.table.pose.orientation")
        if o is None:
            o = [0.0, 0.0, 0.0, 1.0]
        q = Quaternion()
        q.x, q.y, q.z, q.w = o[0], o[1], o[2], o[3]
        return q

    def _move(self, target, timeout=25.0, secs=4):
        if self._use_moveit:
            return self._move_moveit(target, timeout)
        return self._move_direct(target, timeout, secs)

    def _move_moveit(self, target, timeout):
        """Plan + execute via MoveIt (collision-checked). For the real robot."""
        self._clear_octomap()
        self._moveit.move_to_configuration(
            joint_positions=list(target), joint_names=ARM_JOINTS
        )
        deadline = time.time() + timeout
        stable = 0
        err = 99.0
        while time.time() < deadline:
            cur = self._arm_joints()
            err = max(abs(c - t) for c, t in zip(cur, target))
            if err < 0.06:
                stable += 1
                if stable >= 3:
                    return True
            else:
                stable = 0
            time.sleep(0.1)
        yasmin.YASMIN_LOG_WARN(f"move (moveit) timed out (err={err:.3f} rad)")
        return False

    def _move_direct(self, target, timeout, secs=4):
        """Send straight to the controllers (NO collision check). For the sim.
        Returns once the joints arrive or stop improving."""
        self._send_traj(self._arm_ctrl, ARM_JOINTS[1:], target[1:], secs=secs)
        self._send_traj(self._torso_ctrl, ["torso_lift_joint"], [target[0]], secs=secs)
        t0 = time.time()
        err, prev = 99.0, 99.0
        while time.time() - t0 < timeout:
            cur = self._arm_joints()
            err = max(abs(c - t) for c, t in zip(cur, target))
            if err < 0.01 or (time.time() - t0 > secs and prev - err < 0.001):
                time.sleep(0.2)
                return err < 0.05
            prev = err
            time.sleep(0.25)
        yasmin.YASMIN_LOG_WARN(f"move (direct) timed out (err={err:.3f})")
        return False

    def _send_traj(self, client, joint_names, positions, secs=4):
        if not client.wait_for_server(timeout_sec=5.0):
            return None
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions]
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        traj.points = [pt]
        goal.trajectory = traj
        return client.send_goal_async(goal)

    def _run_traj(self, client, joint_names, positions, secs=2, timeout=15.0):
        """Send a trajectory and wait for the controller's result, which arrives in
        sim time (so a slow Gazebo can't make us move on early)."""
        fut = self._send_traj(client, joint_names, positions, secs)
        if fut is None:
            return False
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 5.0:
            time.sleep(0.05)
        gh = fut.result() if fut.done() else None
        if gh is None or not gh.accepted:
            return False
        res = gh.get_result_async()
        while not res.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        return res.done()

    def _gripper_cmd(self, positions):
        if not self._run_traj(self._gripper, GRIPPER_JOINTS, positions, secs=2, timeout=10.0):
            yasmin.YASMIN_LOG_WARN("gripper controller did not report back")
        time.sleep(0.5)

    def _wait_for_tf(self, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                self._tf.lookup_transform(
                    "base_footprint", "map", ROS2Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
                return True
            except Exception:
                time.sleep(0.2)
        return False

    def _target_in_base(self, point_map):
        try:
            tf = self._tf.lookup_transform(
                "base_footprint", "map", ROS2Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            ps = PointStamped()
            ps.header.frame_id = "map"
            ps.header.stamp = tf.header.stamp
            ps.point = point_map
            return do_transform_point(ps, tf).point
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"TF map->base_footprint failed: {e}")
            return None

    def _get_ee(self):
        try:
            tf = self._tf.lookup_transform(
                "base_footprint", "gripper_grasping_frame", ROS2Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            return tf.transform.translation
        except Exception:
            return None

    def _transform(self, point, src, dst):
        try:
            tf = self._tf.lookup_transform(
                dst, src, ROS2Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            ps = PointStamped()
            ps.header.frame_id = src
            ps.header.stamp = tf.header.stamp
            ps.point = point
            return do_transform_point(ps, tf).point
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"TF {src}->{dst} failed: {e}")
            return None

    def _base_pose(self, frame):
        """(x, y, yaw) of base_footprint in `frame`, or None."""
        try:
            t = self._tf.lookup_transform(
                frame, "base_footprint", ROS2Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            ).transform
        except Exception:
            return None
        q = t.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.translation.x, t.translation.y, yaw

    def _holding(self, sent):
        """True if the fingers stopped on something short of where they were sent."""
        opening = sum(self._joints.get(j, 0.0) for j in GRIPPER_JOINTS)
        yasmin.YASMIN_LOG_INFO(f"Gripper opening after lift: {opening:.3f} (sent to {2 * sent:.3f})")
        return opening > 2 * sent + 0.003

    # ── arm model (URDF), depth camera and head ───────────────────────────
    def _setup_ik(self):
        if getattr(self, "_ik_ready", False):
            return
        self._ik = None
        self._urdf = None
        self._depth = None
        self._info = None
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        cam = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
        )
        self._mn.create_subscription(String, "/robot_description", self._urdf_cb, latched)
        self._mn.create_subscription(Image, DEPTH_TOPIC, self._depth_cb, cam)
        self._mn.create_subscription(CameraInfo, INFO_TOPIC, self._info_cb, cam)
        self._head_ctrl = ActionClient(
            self._mn, FollowJointTrajectory, "/head_controller/follow_joint_trajectory",
        )
        self._amcl_cli = self._mn.create_client(Empty, "/request_nomotion_update")
        self._ik_ready = True

    def _urdf_cb(self, msg):
        self._urdf = msg.data

    def _depth_cb(self, msg):
        self._depth = msg

    def _info_cb(self, msg):
        self._info = msg

    def _urdf_from_param(self):
        cli = self._mn.create_client(GetParameters, "/robot_state_publisher/get_parameters")
        if not cli.wait_for_service(timeout_sec=2.0):
            return None
        fut = cli.call_async(GetParameters.Request(names=["robot_description"]))
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 3.0:
            time.sleep(0.05)
        if not fut.done() or fut.result() is None or not fut.result().values:
            return None
        return fut.result().values[0].string_value or None

    def _get_ik(self):
        """IK built from the robot's own URDF, or None if it can't be had."""
        if self._ik is not None:
            return self._ik
        t0 = time.time()
        while self._urdf is None and time.time() - t0 < 5.0:
            time.sleep(0.1)
        urdf = self._urdf or self._urdf_from_param()
        if not urdf:
            yasmin.YASMIN_LOG_WARN("No /robot_description: can't build the arm model")
            return None
        try:
            self._ik = ArmIK(urdf, ARM_JOINTS)
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"Arm model from URDF failed: {e}")
        return self._ik

    def _ik_matches_tf(self, ik):
        """The model must put the gripper where TF says it is (arm at rest)."""
        ee = self._get_ee()
        if ee is None:
            return False
        p = ik.fk(self._arm_joints())[:3, 3]
        err = math.dist(p, (ee.x, ee.y, ee.z))
        yasmin.YASMIN_LOG_INFO(f"Arm model check: URDF gripper vs TF gripper = {100 * err:.1f} cm")
        return err < 0.01

    def _amcl_settle(self):
        """Let AMCL re-localise from the laser without moving, so map->odom corrects what
        wheel odometry got wrong."""
        if self._amcl_cli.wait_for_service(timeout_sec=1.0):
            for _ in range(3):
                self._amcl_cli.call_async(Empty.Request())
                time.sleep(0.4)
        time.sleep(0.6)

    def _turn_by(self, angle, timeout=20.0):
        """Turn in place by `angle` rad as odometry measures it."""
        start = self._base_pose("odom")
        if start is None:
            return
        t = Twist()
        deadline = time.time() + timeout
        while time.time() < deadline:
            cur = self._base_pose("odom")
            if cur is None:
                break
            turned = math.atan2(math.sin(cur[2] - start[2]), math.cos(cur[2] - start[2]))
            left = angle - turned
            if abs(left) < 0.01 or left * angle < 0:
                break
            # the base ignores very small commands, so never go below ~0.3 rad/s
            t.angular.z = math.copysign(max(0.3, min(0.4, 0.8 * abs(left))), left)
            self._cmd_vel.publish(t)
            time.sleep(0.05)
        self._cmd_vel.publish(Twist())
        time.sleep(0.5)

    def _face(self, point_map, tol=0.03):
        """Turn the base until the object is straight ahead as AMCL (map frame) sees it.
        Wheel odometry over-reports turning on the spot in the sim (22 deg real for 32 deg
        believed), so turn, let AMCL settle, and correct what is left."""
        for _ in range(4):
            tb = self._target_in_base(point_map)
            if tb is None:
                return
            err = math.atan2(tb.y, tb.x)
            if not math.isfinite(err) or abs(err) < tol:
                return
            self._turn_by(err)
            self._amcl_settle()

    def _look_at(self, x, y, z):
        """Point the head at (x, y, z) in base_footprint, then wait for a fresh depth frame
        (none if the head couldn't be pointed, so an old frame is never measured)."""
        self._depth = None
        try:
            h = self._tf.lookup_transform(
                "base_footprint", "head_2_link", ROS2Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            ).transform.translation
        except Exception:
            return
        pan = math.atan2(y - h.y, x - h.x)
        tilt = -math.atan2(h.z - z, math.hypot(x - h.x, y - h.y))
        pan, tilt = max(-1.2, min(1.2, pan)), max(-0.95, min(0.7, tilt))
        self._run_traj(self._head_ctrl, ["head_1_joint", "head_2_joint"], [pan, tilt], secs=2)
        time.sleep(0.5)
        self._depth = None
        t0 = time.time()
        while self._depth is None and time.time() - t0 < 3.0:
            time.sleep(0.1)

    def _depth_points(self, step=4):
        """The latest depth image as points in base_footprint (every `step`-th pixel),
        with the image row of each point and the image height. (None, None, 0) if none."""
        d, info = self._depth, self._info
        if d is None or info is None:
            return None, None, 0
        if d.encoding == "32FC1":
            img = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.step // 4)
        elif d.encoding == "16UC1":
            img = np.frombuffer(d.data, dtype=np.uint16).reshape(d.height, d.step // 2) / 1000.0
        else:
            return None, None, 0
        vs, us = np.mgrid[0:d.height:step, 0:d.width:step]
        z = img[vs, us].astype(np.float64)
        ok = np.isfinite(z) & (z > 0.2) & (z < 3.0)
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        cam = np.stack([(us[ok] - cx) * z[ok] / fx, (vs[ok] - cy) * z[ok] / fy, z[ok]], axis=1)
        try:
            t = self._tf.lookup_transform(
                "base_footprint", d.header.frame_id, ROS2Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            ).transform
        except Exception:
            return None, None, 0
        qx, qy, qz, qw = t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w
        R = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ])
        pts = cam @ R.T + np.array([t.translation.x, t.translation.y, t.translation.z])
        return pts, vs[ok], d.height

    def _front_view(self, r):
        """From the head depth camera: (x of the nearest thing in the base's driving
        corridor, height of that surface, whether that edge sits on the bottom of the
        image so the real edge may be nearer), in base_footprint."""
        pts, rows, height = self._depth_points()
        if pts is None:
            return None, None, False
        # in front of the base (past its own front + the tucked arm), above the floor
        m = (np.abs(pts[:, 1]) < r + 0.05) & (pts[:, 2] > 0.10) & (pts[:, 0] > r + 0.10)
        sel, rows = pts[m], rows[m]
        if len(sel) < 20:
            return None, None, False
        edge = float(np.percentile(sel[:, 0], 2))
        near = sel[:, 0] < edge + 0.10
        # on the image bottom, or at the near cut above: the real edge may be nearer
        clipped = bool(np.any(rows[sel[:, 0] < edge + 0.02] >= height - 8)) or edge < r + 0.12
        # the table top is the flat surface there: the most common height (the front face
        # spreads below it, anything standing near the edge spreads above it)
        zs = sel[near, 2]
        h, e = np.histogram(zs, bins=np.arange(zs.min(), zs.max() + 0.02, 0.01))
        k = int(np.argmax(h))
        return edge, float(np.median(zs[(zs >= e[k] - 0.005) & (zs < e[k + 1] + 0.005)])), clipped

    def _edge_line(self, table_z, r):
        """The table's near edge in the latest depth image (head looking down in front of
        the base) as a line x = a + b*y in base_footprint, from the table top's vertical
        front face, which only exists right at the edge (the arm is higher, so it never
        counts). None if the face isn't seen: the nearest table-top points would only be
        where the view ends."""
        pts, _, _ = self._depth_points()
        if pts is None:
            return None
        ahead = (np.abs(pts[:, 1]) < 0.45) & (pts[:, 0] > r + 0.05)
        face = pts[ahead & (pts[:, 2] > table_z - 0.045) & (pts[:, 2] < table_z - 0.008)]
        # a face spans its height; the top's edge or a few stray points don't
        if (len(face) < 30 or np.ptp(face[:, 1]) < 0.15
                or np.percentile(face[:, 2], 95) - np.percentile(face[:, 2], 5) < 0.015):
            return None
        # the line most face points lie within 1 cm of (the table's end face, square to the
        # front, or something in the way can't pull it), then least squares on those
        ys, xs = face[:, 1], face[:, 0]
        rng = np.random.default_rng(0)
        best, a, b = 0, 0.0, 0.0
        for _ in range(200):
            i, j = rng.choice(len(ys), 2, replace=False)
            if abs(ys[j] - ys[i]) < 0.05:
                continue
            bb = (xs[j] - xs[i]) / (ys[j] - ys[i])
            aa = xs[i] - bb * ys[i]
            n_in = int((np.abs(xs - (aa + bb * ys)) < 0.01).sum())
            if abs(bb) < 1.5 and n_in > best:
                best, a, b = n_in, aa, bb
        for _ in range(2):
            inl = np.abs(xs - (a + b * ys)) < 0.01
            if (inl.sum() < 30 or np.ptp(ys[inl]) < 0.15
                    or np.percentile(face[inl, 2], 95) - np.percentile(face[inl, 2], 5) < 0.015):
                return None
            a, b = np.linalg.lstsq(np.stack([np.ones(inl.sum()), ys[inl]], 1), xs[inl], rcond=None)[0]
        return float(a), float(b)

    def _edge_x(self, line):
        """x of the nearest point of an edge line in the base's driving corridor: the same
        thing _front_view measures, so the two can be compared."""
        a, b = line
        return a - abs(b) * (self._r + 0.05)

    def _look_edge(self, x_edge, table_z):
        """Aim the head at the table edge expected at x_edge, then fit the edge line."""
        self._look_at(max(self._r + 0.15, x_edge), 0.0, table_z - 0.02)
        line = self._edge_line(table_z, self._r)
        if line is None:   # the edge may be below the view: look as far down as the head goes
            self._look_at(self._r, 0.0, table_z - 0.30)
            line = self._edge_line(table_z, self._r)
        if line is None:   # still below it (base close, torso up): look down to the left, away
            # from the arm, where the view reaches nearer and the face crosses many rows
            self._look_at(self._r, 0.30, table_z - 0.30)
            line = self._edge_line(table_z, self._r)
        return line

    @staticmethod
    def _moved(line0, line1):
        """How the base moved between two views of the same straight table edge, as an arc
        (a differential drive that drifts turns gradually and can't slip sideways): the
        chord length t and the turn phi (rad, + = left). The chord points phi/2 off the
        old heading."""
        (a0, b0), (a1, b1) = line0, line1
        phi = math.atan(b1) - math.atan(b0)
        h0 = math.hypot(1.0, b0)
        d0, d1 = a0 / h0, a1 / math.hypot(1.0, b1)   # distance from the base to the edge
        return (d0 - d1) * h0 / (math.cos(phi / 2.0) - b0 * math.sin(phi / 2.0)), phi

    def _measure_object(self, g, table_z, radius=0.08):
        """The object as the depth camera sees it (head looking at it): the points
        standing on the table within 8 cm of where it was detected. Returns its centre
        x, y and width at the height to grasp it (half its height, clear of the table),
        that height and its top z, in base_footprint; None if nothing is there."""
        pts, _, _ = self._depth_points(step=1)
        if pts is None:
            return None
        pts = pts[(pts[:, 2] > table_z + 0.01) & (pts[:, 2] < table_z + 0.40)]
        cx, cy, found = g[0], g[1], None
        for _ in range(3):   # re-centre on what was found, in case the detection was off
            obj = pts[np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) < radius]
            if len(obj) < 50:
                return found
            top = float(np.percentile(obj[:, 2], 99))
            z = table_z + min(max(0.5 * (top - table_z), 0.045), 0.12)
            band = obj[np.abs(obj[:, 2] - z) < 0.02]
            if len(band) < 20:
                return found
            lo, hi = np.percentile(band[:, 1], [1, 99])
            width = float(hi - lo)
            # the camera sees the front half: centre = front surface + radius
            cx = float(np.percentile(band[:, 0], 1)) + width / 2.0
            cy = float(lo + hi) / 2.0
            found = (cx, cy, z, top, width)
        return found

    def _in_the_way(self, c, width, table_z, edge):
        """x of the nearest thing standing on the table in the gripper's lane between
        the table edge and the object, or None if the lane is clear."""
        pts, _, _ = self._depth_points()
        if pts is None:
            return None
        m = ((np.abs(pts[:, 1] - c[1]) < 0.07) & (pts[:, 2] > table_z + 0.015)
             & (pts[:, 2] < table_z + 0.40) & (pts[:, 0] > edge)
             & (pts[:, 0] < c[0] - width / 2.0 - 0.01))
        return float(np.percentile(pts[m, 0], 5)) if m.sum() >= 15 else None

    def _drive_to(self, dist, timeout=30.0):
        """Drive straight `dist` m (negative = backwards), closed loop on odom so it
        holds in sim time too; stops if blocked. Returns the distance covered (>= 0)."""
        start = self._base_pose("odom")
        goal = abs(dist)
        if start is None or goal < 0.005:
            return 0.0
        sgn = 1.0 if dist > 0 else -1.0
        driven, last_t, last_driven = 0.0, time.time(), 0.0
        t = Twist()
        deadline = time.time() + timeout
        while time.time() < deadline:
            cur = self._base_pose("odom")
            if cur is None:
                break
            driven = math.hypot(cur[0] - start[0], cur[1] - start[1])
            if driven >= goal - 0.005:
                break
            if driven - last_driven > 0.005:
                last_t, last_driven = time.time(), driven
            elif time.time() - last_t > 2.0:
                yasmin.YASMIN_LOG_WARN(f"Drive stopped after {driven:.2f} of {goal:.2f} m: base is not moving (blocked?)")
                break
            # the base ignores very small commands (it stalled at ~0.02 m/s), so stay above 0.06
            t.linear.x = sgn * max(0.06, min(0.08, 0.6 * (goal - driven)))
            self._cmd_vel.publish(t)
            time.sleep(0.05)
        self._cmd_vel.publish(Twist())
        time.sleep(0.5)
        cur = self._base_pose("odom")
        return driven if cur is None else math.hypot(cur[0] - start[0], cur[1] - start[1])

    def _target_now(self, point_odom, depth):
        tb = self._transform(point_odom, "odom", "base_footprint")
        return None if tb is None else np.array([tb.x + depth, tb.y, tb.z])

    # ── collision checks along joint-space moves ────────────────────────────
    # The controllers move every joint in a straight line from start to goal, so a
    # move is safe only if every pose along that line is.
    def _arm_points(self, ik, q, holding):
        """(point, clearance it needs above the table, whether it must stay out of the
        robot's body) from the shoulder to the fingertips, in base_footprint."""
        o = ik.joint_origins(q)
        T = ik.fk(q)
        tip, a = T[:3, 3], T[:3, 0]
        names = ("arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint")
        chain = [(o[n], KEEP_UP[n], n not in ("arm_2_joint", "arm_3_joint")) for n in names]
        chain.append((tip, GRIPPER_CLEARANCE, True))
        pts = [(p + s * (n - p), mp + s * (mn - mp), bp and bn)
               for (p, mp, bp), (n, mn, bn) in zip(chain[:-1], chain[1:]) for s in (0.0, 0.25, 0.5, 0.75)]
        pts += [(tip, GRIPPER_CLEARANCE, True), (tip + 0.05 * a, GRIPPER_CLEARANCE, True)]   # fingertips
        if holding:   # bottom of the held object, which sits mid-pad
            pts.append((tip + self._pad_mid * a - np.array([0.0, 0.0, 0.07]), 0.01, True))
        return pts

    def _hits(self, p, clearance, body, torso, edge, table_z):
        if p[0] > edge - 0.05 and p[2] < table_z + clearance:
            return True                        # into or under the table
        if not body:
            return False                       # shoulder links: part of the body anyway
        if -0.22 < p[0] < 0.10 and abs(p[1]) < 0.18 and 0.24 < p[2] < 1.15 + torso:
            return True                        # the robot's torso column
        return math.hypot(p[0], p[1]) < self._r + 0.04 and p[2] < 0.40   # the base

    def _path_ok(self, ik, qs, edge, table_z, holding=True, n=25):
        """True if the straight joint moves through the waypoints qs keep the arm out
        of the table (edge, table_z in the current base frame), out of everything else
        the camera saw standing on it, and out of the robot's own body."""
        # the obstacles were measured with the base where the table edge was at _edge0
        obs = self._obstacles + np.array([edge - self._edge0, 0.0, 0.0])
        for qa, qb in zip(qs[:-1], qs[1:]):
            qa, qb = np.asarray(qa, dtype=float), np.asarray(qb, dtype=float)
            for s in np.linspace(0.0, 1.0, n):
                q = qa + s * (qb - qa)
                pts = self._arm_points(ik, q, holding)
                if any(self._hits(p, c, b, q[0], edge, table_z) for p, c, b in pts):
                    return False
                if len(obs):
                    P = np.array([p for p, _, _ in pts])
                    C = np.array([c for _, c, _ in pts])
                    d2 = ((P[:, None, :] - obs[None, :, :]) ** 2).sum(axis=2)
                    if np.any(d2.min(axis=1) < C ** 2):
                        return False
        return True

    @staticmethod
    def _keep(table_z, edge, margin=True):
        """IK height limits for the arm parts that are past the table edge. With the margin
        they are 1 cm higher and start 5 cm sooner than _hits checks, so an IK answer
        doesn't sit right on that line; without it they are exactly what _hits checks."""
        m = 0.01 if margin else 0.0
        return {n: (table_z + c + m, edge - 0.05 - 5 * m) for n, c in KEEP_UP.items()}

    @staticmethod
    def _straight(ik, qa, qb, tip, side=0.002, tol=0.006):
        """Whether turning the joints evenly from qa to qb (as the controller does) keeps
        the gripper on the straight line between them, at its frame and at the fingertips
        (tip m ahead): within `side` sideways, as the fingers have only a few mm either
        side of the object, and within tol any way (no swing of the wrist)."""
        qa, qb = np.asarray(qa, float), np.asarray(qb, float)
        Ta, Tb = ik.fk(qa), ik.fk(qb)
        a = Ta[:3, 0]
        lat = np.array([-a[1], a[0], 0.0]) / max(math.hypot(a[0], a[1]), 1e-6)
        for s in np.linspace(0.0, 1.0, 9)[1:-1]:
            Tm = ik.fk(qa + s * (qb - qa))
            for off in (0.0, tip):
                line = (1 - s) * (Ta[:3, 3] + off * Ta[:3, 0]) + s * (Tb[:3, 3] + off * Tb[:3, 0])
                dv = Tm[:3, 3] + off * Tm[:3, 0] - line
                if abs(dv @ lat) > side or np.linalg.norm(dv) > tol:
                    return False
        return True

    def _look_past_arm(self, ik, R0, c, width, top, table_z, edge):
        """The object measured directly once the base has driven in. The arm out in front hides
        it from the head camera, so move the gripper aside (right, else up or left), look, then bring the
        arm back the same way (the reach-in starts from the pose the plan checked). The object's
        (centre x, y) if it is seen whole near c; () if the camera got a clear look but it isn't
        there; None if the arm couldn't be moved out of the way."""
        q0 = self._arm_joints()
        ee = ik.fk(q0)[:3, 3]
        keeps = (self._keep(table_z, edge), self._keep(table_z, edge, margin=False))
        tip = ik.pads[1] if ik.pads else 0.10
        for off in ((0.0, -0.20, 0.0), (0.0, -0.20, 0.10), (0.0, 0.15, 0.0), (0.0, 0.0, 0.30)):
            seg = self._line_to(ik, q0, ee + np.array(off), R0, keeps, (0.003,), tip, edge, table_z)
            if seg is None:
                continue
            for q in seg:
                self._move(list(q), timeout=10.0, secs=2)
            self._look_at(*c)
            m = self._measure_object(c, table_z) or self._measure_object(c, table_z, radius=0.15)
            pts = self._depth_points(step=1)[0] if m is None else None
            # bare table top right where the object stood (an object there would hide it): the
            # camera did see the spot, and it's empty
            bare = pts is not None and int(np.count_nonzero(
                (np.hypot(pts[:, 0] - c[0], pts[:, 1] - c[1]) < 0.02) & (np.abs(pts[:, 2] - table_z) < 0.01))) >= 40
            for q in list(reversed(seg[:-1])) + [q0]:   # back the same way
                self._move(list(q), timeout=10.0, secs=2)
            # a partly hidden object measures narrower, or shorter, and off-centre: only a
            # whole one counts
            if (m is not None and m[4] > 0.95 * width and abs(m[3] - top) < 0.015
                    and math.hypot(m[0] - c[0], m[1] - c[1]) < 0.06):
                return m[0], m[1]
            if bare:
                return ()   # nothing standing there at all
        return None

    def _reach_in(self, ik, q, g, R0, standoff, carry_z, high, edge, table_z):
        """Arm poses from the carry pose q to the object at g: above the start if carried
        over the object's top, the start (standoff short of it), then in 3 cm steps to it.
        Each one is solved from the one before, tightest first (1.5 mm with the IK margin,
        else the path check's own limits, else 3 mm); every move must pass the path check
        and keep the gripper on a straight line (no swing of the wrist into the object).
        The plan checks the reach-in with this and the reach-in runs it, so they agree.
        None if a pose can't be reached."""
        start = np.array([g[0] - standoff, g[1], g[2]])
        n = max(2, int(math.ceil(standoff / 0.03)))
        ws = (([np.array([start[0], start[1], carry_z])] if high else []) + [start]
              + [start + k / n * (g - start) for k in range(1, n + 1)])
        keeps = (self._keep(table_z, edge), self._keep(table_z, edge, margin=False))
        tip = ik.pads[1] if ik.pads else 0.10
        qs = []
        for i, w in enumerate(ws):
            tols = (0.003,) if high and i == 0 else (0.0015, 0.003)
            # after the first pose each step is short: solve from the pose before only (a
            # random restart lands on a far-off posture, which would swing anyway)
            seg = self._line_to(ik, q, w, R0, keeps, tols, tip, edge, table_z, restarts=4 if i == 0 else 0)
            if seg is None:
                return None
            qs += seg
            q = seg[-1]
        return qs

    def _line_to(self, ik, q, w, R0, keeps, tols, tip, edge, table_z, depth=0, restarts=4):
        """Joint poses that take the gripper from where q has it to w in a straight line:
        one move if the arm can, else via the halfway point (twice at most, only when the
        arm can get there but would swing). None if it can't."""
        swing = False
        for tp in tols:
            for kp in keeps:
                qn = ik.solve(q, w, R0, kp, tol_pos=tp, restarts=restarts)
                if qn is None or not self._path_ok(ik, [q, qn], edge, table_z, False):
                    continue
                if self._straight(ik, q, qn, tip):
                    return [qn]
                swing = True
        if not swing or depth >= 2:
            return None
        m = (ik.fk(q)[:3, 3] + w) / 2.0
        a = self._line_to(ik, q, m, R0, keeps, tols, tip, edge, table_z, depth + 1, restarts)
        b = self._line_to(ik, a[-1], w, R0, keeps, tols, tip, edge, table_z, depth + 1, restarts) if a else None
        return a + b if b else None

    def _plan(self, ik, q0, g, R0, edge, table_z, max_drive, standoff, carry_z, high):
        """(drive distance, [q0, ..., q_carry]) such that the arm can get from where it
        is to the carry pose without touching the table or the robot, and from there
        reach the object after the drive. None if there is no such plan."""
        x_pre = ik.fk(INIT_JOINTS_RIGHT)[0, 3]
        want = max(0.0, g[0] - standoff - x_pre)
        for d in sorted({min(want, max_drive), max_drive}):
            shift = np.array([d, 0.0, 0.0])
            carry = np.array([g[0] - standoff, g[1], carry_z]) - shift
            init, keep = np.array(INIT_JOINTS_RIGHT), self._keep(table_z, edge)
            # also the answer nearest the pre-grasp pose: from there the arm moves the least
            near = ik.solve(init, carry, R0, keep)
            tried, options = [], []
            for q_carry in ([near] if near is not None else []) + list(ik.solutions(q0, carry, R0, keep, give_up=40)):
                if any(np.abs(q_carry - t).max() < 0.01 for t in tried):
                    continue   # the same answer again, from another seed
                tried.append(q_carry)
                # the cheap check first: getting the arm out to the carry pose
                path = next((p for p in ([q0, q_carry], [q0, init, q_carry])
                             if self._path_ok(ik, p, edge, table_z)), None)
                if path is not None:
                    options.append((sum(np.abs(b - a).max() for a, b in zip(path, path[1:])), len(options), path))
            # least joint travel first, so the arm doesn't swing round through a mirrored
            # posture that ends up looking the same
            for _, _, path in sorted(options):
                if self._reach_in(ik, path[-1], g - shift, R0, standoff, carry_z, high, edge - d, table_z) is not None:
                    return d, path
        return None

    def _retreat(self, ik, path, driven, edge, table_z, holding):
        """Get the arm back to where it started, retracing the checked path: lift over
        to the carry pose, reverse the base to where that path was planned, then go
        back along it. If the base can't get back far enough, the arm is left out
        rather than swung through the table."""
        q_carry = path[-1]
        back = 0.0
        if not self._path_ok(ik, [self._arm_joints(), q_carry], edge - driven, table_z, holding):
            # back straight out from the table with the arm held as it is, then try again
            back = self._drive_to(-driven)
            if not self._path_ok(ik, [self._arm_joints(), q_carry], edge - (driven - back), table_z, holding):
                yasmin.YASMIN_LOG_WARN("Can't bring the arm back over the table safely: leaving it out")
                return False
        self._move(list(q_carry), timeout=15.0, secs=3)
        back += self._drive_to(-(driven - back))
        route = list(reversed(path))
        # the reverse slips like the drive in did: check against where the edge really is
        edge_now = edge - (driven - back)
        for _ in range(2):
            line = self._look_edge(edge_now, table_z)
            # the base only drove straight in and out: a line turned from the first one is
            # not the front face (with the arm out, the table's end can be most of what's seen)
            if (line is not None and self._line0 is not None
                    and abs(math.atan(line[1]) - math.atan(self._line0[1])) < 0.15):
                edge_now = edge - (self._edge_x(self._line0) - self._edge_x(line))
            if self._path_ok(ik, route, edge_now, table_z, holding) or edge - edge_now < 0.02:
                break
            more = min(edge - edge_now, 0.30)
            yasmin.YASMIN_LOG_INFO(
                f"Base is {edge - edge_now:.2f} m short of where the arm came out: backing off {more:.2f} m more"
            )
            edge_now += self._drive_to(-more)
        if not self._path_ok(ik, route, edge_now, table_z, holding):
            yasmin.YASMIN_LOG_WARN(f"Table edge at {edge_now:.2f} m: leaving the arm out, not tucking")
            return False
        for q in route[1:]:
            self._move(list(q), timeout=20.0, secs=5)
        self._remove_table_box()
        return True

    def execute(self, blackboard) -> str:
        if not bool(self._param("pick_and_place.grasp.enable", True)):
            yasmin.YASMIN_LOG_INFO("grasp.enable=false — skipping grasp.")
            return "succeeded"

        self._setup()
        self._setup_ik()

        obj = blackboard["selected_object"]
        p = getattr(obj, "point", None)
        if p is None or not all(math.isfinite(v) for v in (p.x, p.y, p.z)) or (p.x, p.y, p.z) == (0.0, 0.0, 0.0):
            yasmin.YASMIN_LOG_WARN("Object has no valid 3D point (depth projection failed).")
            return "failed"

        t0 = time.time()
        while not self._joints and time.time() - t0 < 5.0:
            time.sleep(0.1)
        if not self._wait_for_tf():
            yasmin.YASMIN_LOG_WARN("TF not ready — skipping grasp.")
            return "failed"

        tb = self._target_in_base(obj.point)
        if tb is None:
            return "failed"
        yasmin.YASMIN_LOG_INFO(
            f"Grasp target (base_footprint): x={tb.x:.2f} y={tb.y:.2f} z={tb.z:.2f}"
        )
        if math.hypot(tb.x, tb.y) > 2.5 or tb.z < 0.2:
            yasmin.YASMIN_LOG_WARN("Grasp target is not on a table in front of the robot.")
            return "failed"
        # Object held fixed in odom from here on: smooth while the base moves.
        point_odom = self._transform(obj.point, "map", "odom")
        if point_odom is None:
            return "failed"

        depth = float(self._param("pick_and_place.grasp.depth", 0.03))
        standoff = float(self._param("pick_and_place.grasp.standoff", 0.10))
        tol = float(self._param("pick_and_place.grasp.tolerance", 0.03))
        self._r = r = float(self._param("pick_and_place.grasp.base_radius", 0.28))
        self._obstacles, self._edge0, self._pad_mid, self._line0 = np.zeros((0, 3)), 0.0, 0.0, None

        ik = self._get_ik()
        if ik is None or not self._ik_matches_tf(ik):
            yasmin.YASMIN_LOG_WARN("No usable arm model: not grasping")
            return "failed"

        # 1. Arm still tucked: face the object, then measure the free space in front of
        #    the base and the table height with the depth camera.
        facing = bool(self._param("pick_and_place.grasp.face_object", True))
        r_obj = math.hypot(tb.x, tb.y)
        if facing:
            yasmin.YASMIN_LOG_INFO("Grasp 1/7: turn to face the object")
            self._face(obj.point)
            # Turning on the spot keeps the object's distance, and it is now straight ahead.
            # AMCL's position can jump ~10 cm, so don't predict the object from it.
            g = np.array([r_obj + depth, 0.0, tb.z])
        else:
            g = self._target_now(point_odom, depth)
        if g is None:
            return "failed"
        # Measure the table and the object; if the object is still off centre, finish the
        # turn by the bearing the camera measured (odometry is off on turns) and measure again.
        for attempt in range(3):
            # Aim low enough that the bottom of the image sees ~0.3 m in front of the base
            # at table height; if the edge still sits on the image bottom, look lower.
            self._look_at(r + 0.45, g[1] / 2.0, g[2] - 0.10)
            edge, table_z, clipped = self._front_view(r)
            if clipped:
                yasmin.YASMIN_LOG_INFO("Depth camera: table edge is on the image bottom, looking lower")
                self._look_at(r + 0.20, g[1] / 2.0, g[2] - 0.10)
                edge, table_z, clipped = self._front_view(r)
            seen = [self._depth_points()[0]]
            if edge is None:
                yasmin.YASMIN_LOG_WARN("Depth camera: can't see the table in front: not grasping")
                return "failed"
            if clipped:
                yasmin.YASMIN_LOG_WARN("Depth camera: table edge is closer than it can see: assuming it's touching")
                edge = r + 0.05
            yasmin.YASMIN_LOG_INFO(f"Depth camera: table edge at x={edge:.2f} m, top z={table_z:.2f}")
            max_drive = max(0.0, min(float(self._param("pick_and_place.grasp.max_forward", 1.0)),
                                     edge - r - 0.05))

            # Look at the object itself: its real centre, width and height from depth. The
            # detection's point is one pixel of a box and can be centimetres off, while the
            # gripper opens only a little wider than a can.
            self._look_at(g[0], g[1], g[2])
            meas = self._measure_object(g, table_z) or self._measure_object(g, table_z, radius=0.15)
            if meas is None:
                yasmin.YASMIN_LOG_WARN("Depth camera: nothing standing where the object was detected: not grasping")
                return "failed"
            ox, oy, oz, top, width = meas
            yasmin.YASMIN_LOG_INFO(
                f"Object from depth: centre x={ox:.3f} y={oy:+.3f} (detection off by {ox - g[0]:+.3f}, "
                f"{oy - g[1]:+.3f}), {100 * width:.1f} cm wide at z={oz:.2f}, top z={top:.2f}"
            )
            # It must be the detected object, not a neighbour (they stand >= 24 cm apart): after
            # the turn, at the detected distance and about straight ahead.
            if facing:
                off_r, off_a = math.hypot(ox, oy) - (r_obj + depth), math.atan2(oy, ox)
                wrong = abs(off_r) > 0.05 or abs(off_a) > 0.12
            else:
                off_r, off_a = math.hypot(ox - g[0], oy - g[1]), 0.0
                wrong = off_r > 0.12
            if wrong:
                yasmin.YASMIN_LOG_WARN(
                    f"Object found {100 * off_r:+.1f} cm / {math.degrees(off_a):+.1f} deg from where it "
                    f"was detected (clear, hidden or a neighbour?): not grasping"
                )
                return "failed"
            if not facing or abs(off_a) < 0.03 or attempt == 2:
                break
            yasmin.YASMIN_LOG_INFO(f"Object is {math.degrees(off_a):+.1f} deg off centre: turning the rest")
            self._turn_by(off_a)
            g = np.array([math.hypot(ox, oy), 0.0, oz])
        opening = 2.0 * GRIPPER_OPEN[0]
        if width + 0.004 > opening - 0.004:
            yasmin.YASMIN_LOG_WARN(
                f"Object is {100 * width:.1f} cm wide, the gripper opens {100 * opening:.1f} cm: not grasping"
            )
            return "failed"
        centre = np.array([ox, oy, oz])
        # The gripper frame sits at the base of the finger pads: aim it so the object ends
        # up in the middle of the pads (clear of the palm), and start far enough back that
        # the open fingertips stay clear of the object. Pad extent from the URDF.
        lo, hi = ik.pads if ik.pads else (0.0, 0.10)
        self._pad_mid = (lo + hi) / 2.0
        g = centre - np.array([self._pad_mid, 0.0, 0.0])
        standoff = max(standoff, hi - self._pad_mid + width / 2.0 + 0.03)
        yasmin.YASMIN_LOG_INFO(
            f"Finger pads {lo:+.3f}..{hi:+.3f} m from the grasp frame: aiming it "
            f"{self._pad_mid:.3f} m short of the centre, standoff {standoff:.2f} m"
        )
        # Everything else standing on the table, from both views: the arm must keep clear
        # of it. 2 cm voxels; the object itself is left out (the gripper goes around it).
        seen.append(self._depth_points()[0])
        obs = np.vstack([v for v in seen if v is not None] or [np.zeros((0, 3))])
        obs = obs[(obs[:, 2] > table_z + 0.015) & (obs[:, 0] > edge - 0.02)
                  & (np.hypot(obs[:, 0] - ox, obs[:, 1] - oy) > width / 2.0 + 0.03)]
        self._obstacles = (np.unique(np.floor(obs / 0.02), axis=0) + 0.5) * 0.02 if len(obs) else obs
        self._edge0 = edge
        blocker = self._in_the_way(centre, width, table_z, edge)
        if blocker is not None:
            yasmin.YASMIN_LOG_WARN(f"Something stands in front of the object at x={blocker:.2f}: not grasping")
            return "failed"
        c = Point()
        c.x, c.y, c.z = (float(v) for v in centre)
        point_odom = self._transform(c, "base_footprint", "odom")
        if point_odom is None:
            return "failed"
        depth = -self._pad_mid   # point_odom is the object's centre; the gripper frame aims short of it

        if bool(self._param("pick_and_place.grasp.publish_box", True)):
            self._publish_table_box()
        self._clear_octomap()
        time.sleep(0.5)

        # 2. Plan before moving anything. Gripper: the pregrasp roll, pointing straight
        #    ahead and level. Elbow and wrist stay above the table, and every joint move
        #    is checked against the table and the robot's own body. If the table is too
        #    close for the arm to come out, back the base off first.
        yasmin.YASMIN_LOG_INFO("Grasp 2/7: planning the arm")
        R0 = point_forward(ik.fk(INIT_JOINTS_RIGHT)[:3, :3])
        # Carried with the open fingers above the object's top (pads from the URDF), so the
        # drive can't push it if the base drifts or overshoots; else just high enough that a
        # held object's bottom still clears the table.
        pts = getattr(ik, "pad_pts", None)
        pad_h = float(np.abs((pts @ R0.T)[:, 2]).max()) if pts is not None else 0.03
        low_z = max(g[2] + 0.03, table_z + 0.11)
        q0 = np.array(self._arm_joints())
        backed, moved, replans = 0.0, 0.0, 0
        while True:
            plan, backoff, carry_z = None, 0.0, low_z
            for b in (0.0, 0.10, 0.20, 0.30):
                if backed + b > 0.40:
                    break
                for carry_z in (max(low_z, top + pad_h + 0.02), low_z):
                    plan = self._plan(ik, q0, g + np.array([b, 0.0, 0.0]), R0, edge + b, table_z,
                                      max_drive + b, standoff, carry_z, carry_z > top + pad_h)
                    if plan is not None:
                        break
                if plan is not None:
                    backoff = b
                    break
            if plan is None:
                yasmin.YASMIN_LOG_WARN(
                    f"Object out of reach: no safe way to x={g[0]:.2f} y={g[1]:.2f} z={g[2]:.2f} "
                    f"(table edge {edge:.2f} m, top {table_z:.2f} m)"
                )
                return "failed"
            d, path = plan
            if backoff == 0.0:
                # The table edge from here, to measure the drive against (arm still tucked).
                # It's measured closer up than the first look: if it's nearer, plan from it.
                line0 = self._line0 = self._look_edge(edge, table_z)
                if line0 is None:
                    yasmin.YASMIN_LOG_WARN(
                        "Can't see the table edge's front face to measure the drive against: not grasping"
                    )
                    self._remove_table_box()
                    return "failed"
                nearer = self._edge_x(line0) - edge
                if nearer < -0.005:
                    self._edge0 += nearer   # the obstacles stay where they were seen
                    edge += nearer
                    max_drive = max(0.0, min(float(self._param("pick_and_place.grasp.max_forward", 1.0)),
                                             edge - r - 0.05))
                    # the plan may still hold with the edge there; else plan again from it
                    if not (d <= max_drive and self._path_ok(ik, path, edge, table_z) and self._reach_in(
                        ik, path[-1], g - np.array([d, 0.0, 0.0]), R0, standoff, carry_z,
                        carry_z > top + pad_h, edge - d, table_z,
                    ) is not None):
                        if replans < 2:
                            replans += 1
                            yasmin.YASMIN_LOG_INFO(
                                f"Table edge is nearer than first seen, at x={edge:.2f} m: planning again"
                            )
                            continue
                        yasmin.YASMIN_LOG_WARN(f"Table edge at {edge:.2f} m: arm can't come out, not grasping")
                        self._remove_table_box()
                        return "failed"
                break
            yasmin.YASMIN_LOG_INFO(f"Table too close for the arm to come out: backing off {backoff:.2f} m")
            got = self._drive_to(-backoff)
            if got < backoff - 0.03:
                yasmin.YASMIN_LOG_WARN("Could not back off: not grasping")
                return "failed"
            backed += got
            # The back-off slips too, and the edge may have been a guess: measure the edge and
            # the object again from here (arm still tucked), then plan again from what was seen.
            self._look_at(r + 0.45, 0.0, table_z - 0.10)
            seen_edge, _, clipped = self._front_view(r)
            if seen_edge is not None and clipped:
                self._look_at(r + 0.20, 0.0, table_z - 0.10)
                seen_edge, _, clipped = self._front_view(r)
            # an edge on the image bottom is only where the view ends: go by odometry then
            edge = seen_edge if seen_edge is not None and not clipped else edge + got
            c_pred = centre + np.array([got, 0.0, 0.0])
            self._look_at(c_pred[0], c_pred[1], c_pred[2])
            meas = self._measure_object(c_pred, table_z) or self._measure_object(c_pred, table_z, radius=0.15)
            if meas is None or math.hypot(meas[0] - c_pred[0], meas[1] - c_pred[1]) > 0.12:
                yasmin.YASMIN_LOG_WARN("Lost the object after backing off: not grasping")
                return "failed"
            # the obstacles moved with the base: by as much as the object did (odometry slips)
            moved += meas[0] - centre[0]
            self._edge0 = edge - moved
            yasmin.YASMIN_LOG_INFO(
                f"After backing off: table edge at x={edge:.2f} m, object {meas[0] - c_pred[0]:+.3f} m "
                f"from where odometry put it"
            )
            centre = np.array([meas[0], meas[1], centre[2]])
            g = centre - np.array([self._pad_mid, 0.0, 0.0])
            max_drive = max(0.0, min(float(self._param("pick_and_place.grasp.max_forward", 1.0)),
                                     edge - r - 0.05))
        yasmin.YASMIN_LOG_INFO(
            f"Grasp 3/7: plan — drive {d:.2f} m (free space {edge - r - 0.05:.2f} m), "
            f"arm via {len(path) - 2} waypoint(s) at z={carry_z:.2f} "
            f"({'over' if carry_z > top + pad_h else 'below'} the object's top {top:.2f}), then reach in"
        )

        # 3. Arm out along the checked path (base still back from the table), then drive.
        self._gripper_cmd(GRIPPER_OPEN)
        for i, q in enumerate(path[1:], start=1):
            if not self._move(list(q), timeout=20.0, secs=5):
                yasmin.YASMIN_LOG_WARN("Arm did not reach the planned pose: not driving")
                for q_back in reversed(path[:i]):   # back along the same checked line
                    self._move(list(q_back), timeout=20.0, secs=5)
                self._remove_table_box()
                return "failed"
        yasmin.YASMIN_LOG_INFO("Grasp 4/7: drive in")
        # Odometry slips, so stop short, see how far the base really got from the table
        # edge, and drive the rest by that (never closer than the base clears the edge).
        driven = self._drive_to(d - 0.15 if d > 0.30 else d)
        # what the edge should show after the drive, and how far off it may be: by odometry
        # alone that's loose (Gazebo's over-reports straight travel by about a third)
        expect, tol = driven, 0.05 + 0.4 * driven
        if driven < d - 0.05:
            seen = self._look_edge(edge - driven, table_z)
            slip = 1.0
            if seen is not None:
                moved = self._edge_x(line0) - self._edge_x(seen)
                rest = min(d - moved, self._edge_x(seen) - r - 0.05)
                # the wheels slip about as much on the rest of the way: ask odometry for that
                if moved > 0.10:
                    slip = min(1.4, max(0.7, driven / moved))
                yasmin.YASMIN_LOG_INFO(
                    f"Table edge says the base moved {moved:.3f} m so far (odometry {driven:.3f} m): "
                    f"driving the last {max(rest, 0.0):.3f} m (odometry x{slip:.2f})"
                )
            else:
                rest = d - driven
            last = self._drive_to(rest * slip) if rest > 0.01 else 0.0
            if seen is not None:   # then only the short last leg is by odometry
                expect, tol = moved + last / slip, 0.05 + 0.2 * last / slip
            else:
                expect, tol = driven + last, 0.05 + 0.4 * (driven + last)
            driven += last
        keep = self._keep(table_z, edge - driven)

        # 4. Where the object is now. The wheels slip (straight as well as on turns) and the
        #    arm now hides the object from the camera, so measure how far the base really
        #    moved from the table edge, seen beside the arm before and after.
        yasmin.YASMIN_LOG_INFO("Grasp 5/7: check the drive against the table edge, reach in")
        g = self._target_now(point_odom, depth)
        if g is None:
            self._retreat(ik, path, driven, edge, table_z, False)
            return "failed"
        line1 = self._look_edge(edge - driven, table_z)
        if line1 is None:
            yasmin.YASMIN_LOG_WARN(
                "Couldn't see the table edge after the drive, so can't tell how far the base "
                "really moved: not grasping"
            )
            self._retreat(ik, path, driven, edge, table_z, False)
            return "failed"
        t, phi = self._moved(line0, line1)
        odo = driven
        yasmin.YASMIN_LOG_INFO(
            f"Table edge x={line0[0]:.3f}{line0[1]:+.3f}*y before, x={line1[0]:.3f}{line1[1]:+.3f}*y after: "
            f"the base moved {t:.3f} m and turned {math.degrees(phi):+.1f} deg "
            f"(odometry said {odo:.3f} m; expected {expect:.3f} m)"
        )
        if abs(t - expect) > tol or abs(phi) > 0.15:
            yasmin.YASMIN_LOG_WARN(f"That is {t - expect:+.3f} m from what was expected: not grasping")
            self._retreat(ik, path, driven, edge, table_z, False)
            return "failed"
        cx, cy = centre[0] - t * math.cos(phi / 2.0), centre[1] - t * math.sin(phi / 2.0)
        c_now = np.array([math.cos(phi) * cx + math.sin(phi) * cy,
                          -math.sin(phi) * cx + math.cos(phi) * cy, centre[2]])
        g = c_now - np.array([self._pad_mid, 0.0, 0.0])
        # how far the base really is from where the arm came out, as the edge moved
        driven = self._edge_x(line0) - self._edge_x(line1)
        keep = self._keep(table_z, edge - driven)
        # The edge gives the drive to a few mm but the turn only to ~0.4 deg (~7 mm at the
        # object): look at the object itself, with the arm moved out of the camera's view.
        seen = self._look_past_arm(ik, R0, c_now, width, top, table_z, edge - driven)
        if seen:
            yasmin.YASMIN_LOG_INFO(
                f"Object from depth after the drive: centre x={seen[0]:.3f} y={seen[1]:+.3f} "
                f"(the drive estimate was off by {seen[0] - c_now[0]:+.3f}, {seen[1] - c_now[1]:+.3f})"
            )
            c_now = np.array([seen[0], seen[1], c_now[2]])
            g = c_now - np.array([self._pad_mid, 0.0, 0.0])
        else:
            yasmin.YASMIN_LOG_INFO("Couldn't see the whole object past the arm: going by the table edge")
        start = np.array([g[0] - standoff, g[1], g[2]])
        qs = self._reach_in(ik, self._arm_joints(), g, R0, standoff, carry_z, carry_z > top + pad_h,
                            edge - driven, table_z)
        if qs is None:
            yasmin.YASMIN_LOG_WARN(f"Object out of reach after driving {driven:.2f} m")
            self._retreat(ik, path, driven, edge, table_z, False)
            return "failed"
        prev = self._arm_joints()
        for qw in qs:
            step = np.linalg.norm(ik.fk(qw)[:3, 3] - ik.fk(prev)[:3, 3])
            self._move(list(qw), timeout=10.0, secs=2 if step > 0.05 else 1)
            prev = qw
        ee = self._get_ee()
        err = None if ee is None else g - np.array([ee.x, ee.y, ee.z])
        if err is not None:
            yasmin.YASMIN_LOG_INFO(
                f"At object: drove {driven:.2f} m, gripper error dx={err[0]:+.3f} "
                f"dy={err[1]:+.3f} dz={err[2]:+.3f}"
            )
        if err is None or np.linalg.norm(err) > tol:
            yasmin.YASMIN_LOG_WARN("Gripper did not get to the object: not closing it.")
            q_out = ik.solve(self._arm_joints(), start, R0, keep)
            if q_out is not None:
                self._move(list(q_out), timeout=10.0, secs=2)
            self._retreat(ik, path, driven, edge, table_z, False)
            return "failed"

        # 5. Close, lift straight up, check the fingers, pull straight back out.
        yasmin.YASMIN_LOG_INFO("Grasp 6/7: close and lift")
        # Close onto the object, a few mm into it, not all the way: the sim drives the fingers
        # to where they're sent whatever is between them, so sending them far inside the
        # object squeezes it out.
        squeeze = float(self._param("pick_and_place.grasp.squeeze", 0.004))
        sent = min(GRIPPER_OPEN[0], max(GRIPPER_CLOSE[0], width / 2.0 - squeeze))
        self._gripper_cmd([sent, sent])
        yasmin.YASMIN_LOG_INFO(
            f"Gripper opening right after closing: {sum(self._joints.get(j, 0.0) for j in GRIPPER_JOINTS):.3f} "
            f"(sent to {2 * sent:.3f}, object {100 * width:.1f} cm)"
        )
        lift = np.array([0.0, 0.0, 0.10])
        for target in (g + lift, start + lift):
            q_next = ik.solve(self._arm_joints(), target, R0, keep)
            if q_next is not None:
                self._move(list(q_next), timeout=10.0, secs=2)
        holding = self._holding(sent)
        if not holding:
            # the fingers can't tell (the sim puts them where they're sent): is the object
            # still standing where it was?
            still = self._look_past_arm(ik, R0, c_now, width, top, table_z, edge - driven)
            if still == ():
                yasmin.YASMIN_LOG_INFO("The object is no longer on the table: holding it")
                holding = True
            elif still:
                yasmin.YASMIN_LOG_INFO("The object is still standing on the table")

        # 6. Back the way the arm came, then tuck.
        yasmin.YASMIN_LOG_INFO("Grasp 7/7: retract and tuck arm")
        if not self._retreat(ik, path, driven, edge, table_z, True):
            return "failed"
        if not holding:
            yasmin.YASMIN_LOG_WARN("Grasp failed: gripper closed on nothing.")
            return "failed"
        yasmin.YASMIN_LOG_INFO("Grasp complete.")
        return "succeeded"
