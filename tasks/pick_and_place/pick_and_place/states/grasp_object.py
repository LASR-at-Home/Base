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
from geometry_msgs.msg import Twist, PointStamped, Pose, Quaternion
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

from pymoveit2 import MoveIt2


ARM_JOINTS = [
    "torso_lift_joint", "arm_1_joint", "arm_2_joint", "arm_3_joint",
    "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint",
]
GRIPPER_JOINTS = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
GRIPPER_OPEN = [0.044, 0.044]
GRIPPER_CLOSE = [0.010, 0.010]

INIT_JOINTS_RIGHT = [
    0.35, 42 * math.pi / 180, 16 * math.pi / 180, -109 * math.pi / 180,
    105 * math.pi / 180, -60 * math.pi / 180, -56 * math.pi / 180, -108 * math.pi / 180,
]
INIT_JOINTS_LEFT = [
    0.35, -42 * math.pi / 180, 16 * math.pi / 180, 109 * math.pi / 180,
    105 * math.pi / 180, 60 * math.pi / 180, -56 * math.pi / 180, 108 * math.pi / 180,
]

DEFAULT_TABLE_POS = [1.3, 0.0, 0.37]
DEFAULT_TABLE_SIZE = [1.2, 1.6, 0.74]

TUCK_JOINTS = [
    0.15, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.00,
]


class GraspObject(yasmin.State):
    """
    Arm grasp: drive the base into place, STOP, then move only the arm.

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

    def _move(self, target, timeout=25.0):
        if self._use_moveit:
            return self._move_moveit(target, timeout)
        return self._move_direct(target, timeout)

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

    def _move_direct(self, target, timeout):
        """Send straight to the controllers (NO collision check). For the sim."""
        self._send_traj(self._arm_ctrl, ARM_JOINTS[1:], target[1:], secs=4)
        self._send_traj(self._torso_ctrl, ["torso_lift_joint"], [target[0]], secs=4)
        deadline = time.time() + timeout
        err = 99.0
        while time.time() < deadline:
            cur = self._arm_joints()
            err = max(abs(c - t) for c, t in zip(cur, target))
            if err < 0.06:
                time.sleep(0.3)
                return True
            time.sleep(0.1)
        yasmin.YASMIN_LOG_WARN(f"move (direct) timed out (err={err:.3f} rad)")
        return False

    def _send_traj(self, client, joint_names, positions, secs=4):
        if not client.wait_for_server(timeout_sec=5.0):
            return False
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions]
        pt.time_from_start = Duration(sec=int(secs))
        traj.points = [pt]
        goal.trajectory = traj
        client.send_goal_async(goal)
        return True

    def _gripper_cmd(self, positions):
        if not self._gripper.wait_for_server(timeout_sec=5.0):
            yasmin.YASMIN_LOG_WARN("gripper controller unavailable")
            return False
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = GRIPPER_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.time_from_start = Duration(sec=2)
        traj.points = [pt]
        goal.trajectory = traj
        self._gripper.send_goal_async(goal)
        time.sleep(3.0)
        return True

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

    def _drive(self, dist, speed=0.08):
        if abs(dist) < 1e-3:
            return
        t = Twist()
        t.linear.x = speed if dist > 0 else -speed
        end = time.time() + abs(dist) / speed
        while time.time() < end:
            self._cmd_vel.publish(t)
            time.sleep(0.05)
        self._cmd_vel.publish(Twist())

    def execute(self, blackboard) -> str:
        if not bool(self._param("pick_and_place.grasp.enable", True)):
            yasmin.YASMIN_LOG_INFO("grasp.enable=false — skipping grasp.")
            return "succeeded"

        self._setup()

        obj = blackboard["selected_object"]
        if obj is None or getattr(obj, "point", None) is None:
            yasmin.YASMIN_LOG_WARN("No object/point to grasp.")
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

        reach = float(self._param("pick_and_place.grasp.reach", 0.60))
        max_fwd = float(self._param("pick_and_place.grasp.max_forward", 0.45))
        forward = min(tb.x - reach, max_fwd)
        if forward > 0.05:
            yasmin.YASMIN_LOG_INFO(f"Grasp: driving base forward {forward:.2f} m")
            self._drive(forward)
            time.sleep(0.8)
            tb = self._target_in_base(obj.point)
            if tb is None:
                return "failed"
            yasmin.YASMIN_LOG_INFO(
                f"After approach: target x={tb.x:.2f} y={tb.y:.2f} z={tb.z:.2f}"
            )

        if bool(self._param("pick_and_place.grasp.publish_box", True)):
            self._publish_table_box()
        self._clear_octomap()
        time.sleep(0.5)

        self._gripper_cmd(GRIPPER_OPEN)
        config = INIT_JOINTS_LEFT if tb.y > 0.05 else INIT_JOINTS_RIGHT
        yasmin.YASMIN_LOG_INFO("Grasp 1/4: pregrasp pose")
        self._move(config, timeout=25.0)

        yasmin.YASMIN_LOG_INFO("Grasp 2/4: align sideways")
        for _ in range(5):
            ee = self._get_ee()
            tb2 = self._target_in_base(obj.point)
            if ee is None or tb2 is None:
                break
            dy = tb2.y - ee.y
            if abs(dy) < 0.02:
                break
            joints = self._arm_joints()
            joints[1] = max(-1.1, min(1.5, joints[1] + 1.5 * dy))
            self._move(joints, timeout=15.0)

        yasmin.YASMIN_LOG_INFO("Grasp 3/4: lower to object")
        ee = self._get_ee()
        tb3 = self._target_in_base(obj.point) or tb
        if ee is not None:
            dz = tb3.z - ee.z
            joints = self._arm_joints()
            joints[0] = max(0.0, min(0.35, joints[0] + dz))
            self._move(joints, timeout=15.0)

        yasmin.YASMIN_LOG_INFO("Grasp 4/4: close and lift")
        self._gripper_cmd(GRIPPER_CLOSE)
        time.sleep(1.0)
        joints = self._arm_joints()
        joints[0] = min(0.35, joints[0] + 0.10)
        self._move(joints, timeout=15.0)

        yasmin.YASMIN_LOG_INFO("Grasp 5/5: tuck arm for navigation")
        self._move(TUCK_JOINTS, timeout=20.0)

        self._remove_table_box()

        yasmin.YASMIN_LOG_INFO("Grasp complete.")
        return "succeeded"