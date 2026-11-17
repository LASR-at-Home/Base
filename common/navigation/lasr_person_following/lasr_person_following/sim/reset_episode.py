"""
Puts the sim back into a known state between test runs (sim testing only):
deletes scripted people, centres the head, restores Nav2 speed limits and drives the
robot back to the world-frame start pose with Nav2.
"""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters

import tf2_ros
from gazebo_msgs.srv import DeleteEntity
from nav2_msgs.action import BackUp, NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_srvs.srv import Empty
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.duration import Duration


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class ResetEpisode(Node):
    def __init__(self):
        super().__init__("reset_episode")
        self.declare_parameter("x", 1.0)  # 1 m into the room: the spawn pose is at the map edge
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("people", ["test_person", "distractor"])
        self.declare_parameter("max_vel", 0.26)
        # Gazebo's reset_world breaks the TIAGo sim's physics (robot drifts off); off by
        # default, the robot drives back with Nav2 instead
        self.declare_parameter("use_reset_world", False)
        # world-frame waypoints used on the way back (lined up with the doorway, which
        # the controller otherwise clips), visited only if the robot is beyond them
        self.declare_parameter("via_x", [5.2, 3.2])
        self.declare_parameter("via_y", [-1.0, -0.9])
        # map <- world, from the initial pose that nav.launch.py gives AMCL for the
        # robot's spawn pose (the world origin)
        self.declare_parameter("map_from_world", [9.151, -6.340, 2.204])
        self.gt = None
        self.create_subscription(Odometry, "/ground_truth_odom", self._gt_cb, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _gt_cb(self, m):
        self.gt = (m.pose.pose.position.x, m.pose.pose.position.y, _yaw(m.pose.pose.orientation))

    def _stop_navigation(self):
        nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        if not nav.wait_for_server(timeout_sec=5.0):
            return
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time(),
                                                timeout=Duration(seconds=5.0)).transform
        except Exception:
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x, goal.pose.pose.position.y = t.translation.x, t.translation.y
        goal.pose.pose.orientation = t.rotation
        fut = nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        gh = fut.result()
        if gh is not None and gh.accepted:
            c = gh.cancel_goal_async()
            rclpy.spin_until_future_complete(self, c, timeout_sec=5.0)
        stop = self.create_publisher(Twist, "/cmd_vel", 1)
        for _ in range(10):
            stop.publish(Twist())
            time.sleep(0.1)

    def _navigate(self, nav, goal_w, timeout=120.0):
        """NavigateToPose to a world-frame pose; backs up and retries once if Nav2 fails."""
        for attempt in range(2):
            self._localise()  # AMCL drifts on long drives: re-align before each goal
            gx, gy, gyaw = self._world_to_map(*goal_w)
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
            goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
            fut = nav.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            gh = fut.result()
            if gh is None or not gh.accepted:
                continue
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res, timeout_sec=timeout)
            if res.done() and res.result().status == 4:  # SUCCEEDED
                return True
            if not res.done():
                gh.cancel_goal_async()
            self._backup()
        return False

    def _backup(self):
        cli = ActionClient(self, BackUp, "/backup")
        if not cli.wait_for_server(timeout_sec=3.0):
            return
        g = BackUp.Goal()
        g.target.x = 0.3
        g.speed = 0.15
        g.time_allowance = Duration(seconds=6.0).to_msg()
        fut = cli.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() is not None and fut.result().accepted:
            rclpy.spin_until_future_complete(self, fut.result().get_result_async(), timeout_sec=10.0)

    def _world_to_map(self, x, y, yaw):
        mx, my, myaw = self.get_parameter("map_from_world").value
        c, s = math.cos(myaw), math.sin(myaw)
        return mx + c * x - s * y, my + s * x + c * y, yaw + myaw

    def _localise(self) -> bool:
        """Set AMCL's pose from ground truth and wait until /amcl_pose agrees."""
        amcl = {}
        sub = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                       lambda m: amcl.update(p=m.pose.pose.position), 10)
        try:
            for _ in range(5):
                self._set_initial_pose()
                end = time.monotonic() + 4.0
                while time.monotonic() < end:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    if "p" in amcl and self.gt is not None:
                        ex, ey, _ = self._world_to_map(*self.gt)
                        if math.hypot(amcl["p"].x - ex, amcl["p"].y - ey) < 0.3:
                            return True
                amcl.clear()
            self.get_logger().error("AMCL does not agree with ground truth")
            return False
        finally:
            self.destroy_subscription(sub)

    def _set_initial_pose(self):
        end = time.monotonic() + 5.0
        self.gt = None
        while self.gt is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.gt is None:
            return
        mx, my, myaw = self.get_parameter("map_from_world").value
        c, s = math.cos(myaw), math.sin(myaw)
        x, y, yaw = self.gt
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        # zero stamp = "latest": this node runs on wall time, AMCL on sim time
        msg.pose.pose.position.x = mx + c * x - s * y
        msg.pose.pose.position.y = my + s * x + c * y
        msg.pose.pose.orientation.z = math.sin((yaw + myaw) / 2)
        msg.pose.pose.orientation.w = math.cos((yaw + myaw) / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.02
        pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        for _ in range(3):
            pub.publish(msg)
            time.sleep(0.2)
        self.get_logger().info("AMCL re-initialised from ground truth")

    def _call(self, client, req, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def run(self):
        # 1. remove scripted people
        delete = self.create_client(DeleteEntity, "/delete_entity")
        for name in self.get_parameter("people").value:
            self._call(delete, DeleteEntity.Request(name=name))

        # 2. restore Nav2 speed limits (a previous run may have been killed mid-follow)
        v = float(self.get_parameter("max_vel").value)
        self._call(self.create_client(SetParameters, "/controller_server/set_parameters"),
                   SetParameters.Request(parameters=[
                       Parameter("FollowPath.max_vel_x", value=v).to_parameter_msg(),
                       Parameter("FollowPath.max_speed_xy", value=v).to_parameter_msg()]))
        self._call(self.create_client(SetParameters, "/velocity_smoother/set_parameters"),
                   SetParameters.Request(parameters=[
                       Parameter("max_velocity", value=[v, 0.0, 1.0]).to_parameter_msg(),
                       Parameter("min_velocity", value=[-v, 0.0, -1.0]).to_parameter_msg()]))

        # 3. centre the head
        head = ActionClient(self, FollowJointTrajectory, "/head_controller/follow_joint_trajectory")
        if head.wait_for_server(timeout_sec=5.0):
            g = FollowJointTrajectory.Goal()
            g.trajectory.joint_names = ["head_1_joint", "head_2_joint"]
            g.trajectory.points = [JointTrajectoryPoint(
                positions=[0.0, 0.0], time_from_start=Duration(seconds=1.0).to_msg())]
            head.send_goal_async(g)

        # 4. stop any navigation still running from the previous run: pre-empt it with a
        #    goal at the robot's own pose and cancel that (teleporting a moving robot
        #    can make the physics explode)
        self._stop_navigation()

        # 4b. make sure AMCL agrees with ground truth (the launch file's one-off initial
        #     pose can be lost if AMCL was not ready yet); after this, map <- world is the
        #     known constant `map_from_world`, so goals need no TF
        if not self._localise():
            return False

        # 5. teleport (optional, see use_reset_world)
        reset = self.create_client(Empty, "/reset_world")
        if self.get_parameter("use_reset_world").value and self._call(reset, Empty.Request()) is not None:
            time.sleep(1.0)
            if not self._localise():
                return False

        # 5b. forget obstacles marked by people that have since been deleted
        for cm in ("/global_costmap/clear_entirely_global_costmap",
                   "/local_costmap/clear_entirely_local_costmap"):
            self._call(self.create_client(ClearEntireCostmap, cm), ClearEntireCostmap.Request())

        # 6. drive back to the start pose with Nav2
        goal_w = (self.get_parameter("x").value, self.get_parameter("y").value,
                  self.get_parameter("yaw").value)
        nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        nav.wait_for_server(timeout_sec=10.0)
        for vx, vy in zip(self.get_parameter("via_x").value, self.get_parameter("via_y").value):
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.05)
            if self.gt[0] < vx + 0.3:
                continue  # already on the start side of this waypoint
            self.get_logger().info(f"via ({vx}, {vy})")
            self._navigate(nav, (vx, vy, math.pi))
        ok = False
        for attempt in range(5):
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.05)
            err = math.hypot(self.gt[0] - goal_w[0], self.gt[1] - goal_w[1])
            yaw_err = abs(math.atan2(math.sin(self.gt[2] - goal_w[2]), math.cos(self.gt[2] - goal_w[2])))
            self.get_logger().info(f"start pose error {err:.2f} m, {math.degrees(yaw_err):.0f} deg")
            if err < 0.5 and yaw_err < 0.3:
                ok = True
                break
            self._navigate(nav, goal_w)
        # Nav2 often leaves a large heading error: finish with an in-place spin
        spin = ActionClient(self, Spin, "/spin")
        for _ in range(3):
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.05)
            yaw_err = math.atan2(math.sin(goal_w[2] - self.gt[2]), math.cos(goal_w[2] - self.gt[2]))
            if abs(yaw_err) < 0.15 or not spin.wait_for_server(timeout_sec=5.0):
                break
            fut = spin.send_goal_async(Spin.Goal(target_yaw=yaw_err, time_allowance=Duration(seconds=20).to_msg()))
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            if fut.result() is not None and fut.result().accepted:
                res = fut.result().get_result_async()
                rclpy.spin_until_future_complete(self, res, timeout_sec=25.0)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)
        self._localise()
        err = math.hypot(self.gt[0] - goal_w[0], self.gt[1] - goal_w[1])
        yaw_err = abs(math.atan2(math.sin(goal_w[2] - self.gt[2]), math.cos(goal_w[2] - self.gt[2])))
        self.get_logger().info(f"final start pose error {err:.2f} m, {math.degrees(yaw_err):.0f} deg")
        return err < 0.5 and yaw_err < 0.2


def main():
    rclpy.init()
    node = ResetEpisode()
    ok = node.run()
    node.get_logger().info("reset done" if ok else "reset FAILED")
    node.destroy_node()
    rclpy.try_shutdown()
    raise SystemExit(0 if ok else 1)
