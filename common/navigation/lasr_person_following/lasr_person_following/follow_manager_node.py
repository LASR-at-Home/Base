"""
follow_manager_node: drives the robot after the host using Nav2 NavigateToPose goals.

Reads the host estimate from person_tracker (~/host_state) and, while active, keeps
re-sending one goal (a new goal pre-empts the running one without stopping):

  following     host further than follow_distance (+ margin) or walking away:
                goal = `follow_distance` short of where the host will be in `lookahead`
                seconds, on the line robot -> host. The lookahead makes up for the
                controller slowing down as it nears a goal that keeps moving away.
  facing        host close and standing: stay put, only turn to face them
  to_last_seen  tracker lost the host: drive to `follow_distance` short of where they
                were last seen (the state machine decides what to do next)

Nav2's global planner takes care of walls and doorways. While following, Nav2's speed
limits are raised (dynamic parameters) and restored on stop.

Services:  ~/start, ~/stop  (std_srvs/Trigger)
Topics:    ~/status  (std_msgs/String JSON: active, mode, host status, distance, nav, settled)
           ~/goal    (geometry_msgs/PoseStamped, the current goal, for RViz)
"""

import json
import math
import time

import rclpy
import rclpy.signals
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters

import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from std_srvs.srv import Trigger


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class FollowManagerNode(Node):
    def __init__(self):
        super().__init__("follow_manager")
        d = self.declare_parameter
        d("host_state_topic", "/person_tracker/host_state")
        # Following is computed in the tracker's frame (odom: it never jumps when AMCL
        # corrects), goals are handed to Nav2 in its planner frame.
        d("frame", "odom")
        d("planner_frame", "map")
        d("robot_frame", "base_footprint")
        d("follow_distance", 1.0)  # m to keep behind the host
        d("close_margin", 0.5)  # standing host within follow_distance + this: just face them
        d("min_distance", 0.5)  # a goal is never closer than this to the host
        d("moving_speed", 0.25)  # m/s, host speed above which they count as walking
        d("lookahead", 2.0)  # s of host motion to aim ahead
        d("face_tolerance", 0.35)  # rad heading error before turning on the spot
        d("goal_rate", 2.0)  # max goal updates per second
        d("goal_min_move", 0.2)  # m change needed to re-send a goal
        d("goal_min_turn", 0.3)  # rad change needed to re-send a turn-only goal
        d("rate", 10.0)
        # speed limits while following (restored afterwards); empty name = skip
        d("follow_max_speed", 0.7)
        d("controller_node", "/controller_server")
        d("controller_speed_params", ["FollowPath.max_vel_x", "FollowPath.max_speed_xy"])
        d("smoother_node", "/velocity_smoother")
        d("smoother_speed_param", "max_velocity")

        self.p = lambda n: self.get_parameter(n).value
        self.frame = self.p("frame")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.rg = ReentrantCallbackGroup()
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose", callback_group=self.rg)

        self.host = None  # latest host_state dict
        self.create_subscription(String, self.p("host_state_topic"), self._host_cb, 10)
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "~/goal", 10)
        # services block on parameter calls: they must not share the default (mutually
        # exclusive) group with the replies they wait for
        self.create_service(Trigger, "~/start", self._start_srv, callback_group=self.rg)
        self.create_service(Trigger, "~/stop", self._stop_srv, callback_group=self.rg)

        self.active = False
        self.goal_handle = None
        self.last_goal = None  # (x, y, yaw, kind) with kind "drive" or "turn"
        self.last_goal_time = 0.0
        self.nav_state = "idle"
        self.saved_speed = {}
        self.shutting_down = False
        self.create_timer(1.0 / self.p("rate"), self._tick)

    # ------------------------------------------------------------------ io

    def _host_cb(self, msg):
        try:
            self.host = json.loads(msg.data)
        except ValueError:
            pass

    def _lookup(self, target, source):
        try:
            return self.tf_buffer.lookup_transform(target, source, rclpy.time.Time()).transform
        except Exception as e:
            self.get_logger().warn(f"TF {target}<-{source}: {e}", throttle_duration_sec=5.0)
            return None

    def _robot_pose(self):
        t = self._lookup(self.frame, self.p("robot_frame"))
        if t is None:
            return None
        return t.translation.x, t.translation.y, _yaw(t.rotation)

    # ------------------------------------------------------------------ speed params

    def _param_call(self, node, srv_type, req, timeout=2.0):
        kind = "get" if srv_type is GetParameters else "set"
        cli = self.create_client(srv_type, f"{node}/{kind}_parameters", callback_group=self.rg)
        if not cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f"{node} parameter service unavailable")
            return None
        fut = cli.call_async(req)
        if self.shutting_down:  # the executor is gone: spin for the reply ourselves
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.01)
        self.destroy_client(cli)
        return fut.result() if fut.done() else None

    def _raise_speed(self):
        """Humble's DWB only accepts runtime limits up to the speed it was configured with:
        the robot's Nav2 config must allow walking pace (see README)."""
        v = float(self.p("follow_max_speed"))
        if v <= 0:
            return
        ctrl, names = self.p("controller_node"), list(self.p("controller_speed_params"))
        if ctrl and names:
            res = self._param_call(ctrl, GetParameters, GetParameters.Request(names=names))
            if res is not None:
                self.saved_speed[ctrl] = [
                    Parameter(n, value=pv.double_value) for n, pv in zip(names, res.values)
                    if pv.type == ParameterType.PARAMETER_DOUBLE]
                self._param_call(ctrl, SetParameters, SetParameters.Request(parameters=[
                    Parameter(n, value=v).to_parameter_msg() for n in names]))
        sm, sp = self.p("smoother_node"), self.p("smoother_speed_param")
        if sm and sp:
            res = self._param_call(sm, GetParameters, GetParameters.Request(names=[sp, "min_velocity"]))
            if (res is not None and len(res.values) == 2
                    and res.values[0].type == ParameterType.PARAMETER_DOUBLE_ARRAY):
                mx = list(res.values[0].double_array_value)
                mn = list(res.values[1].double_array_value)
                self.saved_speed[sm] = [Parameter(sp, value=mx), Parameter("min_velocity", value=mn)]
                mx2, mn2 = list(mx), list(mn)
                mx2[0], mn2[0] = max(mx[0], v), min(mn[0], -v)
                self._param_call(sm, SetParameters, SetParameters.Request(parameters=[
                    Parameter(sp, value=mx2).to_parameter_msg(),
                    Parameter("min_velocity", value=mn2).to_parameter_msg()]))
        self.get_logger().info(f"Nav2 speed limit raised to {v} m/s for following")

    def _restore_speed(self):
        for node, params in self.saved_speed.items():
            self._param_call(node, SetParameters, SetParameters.Request(
                parameters=[pp.to_parameter_msg() for pp in params]))
        if self.saved_speed:
            self.get_logger().info("Nav2 speed limits restored")
        self.saved_speed = {}

    # ------------------------------------------------------------------ services

    def _start_srv(self, req, res):
        if not self.nav.wait_for_server(timeout_sec=2.0):
            res.success, res.message = False, "navigate_to_pose not available"
            return res
        self.last_goal = None
        if not self.active:
            self._raise_speed()
        self.active = True
        res.success, res.message = True, "following"
        self.get_logger().info("follow: start")
        return res

    def _stop_srv(self, req, res):
        self._stop()
        res.success, res.message = True, "stopped"
        return res

    def _stop(self):
        was_active = self.active
        self.active = False
        self._cancel_goal()
        if was_active:
            self._restore_speed()
            self.get_logger().info("follow: stop")

    def _cancel_goal(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.last_goal = None
        self.nav_state = "idle"

    # ------------------------------------------------------------------ main loop

    def _tick(self):
        status = {"active": self.active, "mode": "idle", "host": None, "distance": None}
        host, robot = self.host, self._robot_pose()
        if host is not None:
            status["host"] = host.get("status")
        if self.active and host is not None and robot is not None and host.get("x") is not None:
            status.update(self._follow(host, robot))
        status["nav"] = self.nav_state
        # settled: close to a standing host and no longer driving (safe to talk to them)
        status["settled"] = status["mode"] == "facing" and self.nav_state != "active"
        self.status_pub.publish(String(data=json.dumps(status)))

    def _follow(self, host, robot):
        rx, ry, ryaw = robot
        hx, hy = host["x"], host["y"]
        hvx, hvy = host["vx"], host["vy"]
        lost = host["status"] == "lost"
        speed = 0.0 if lost else math.hypot(hvx, hvy)
        dist = math.hypot(hx - rx, hy - ry)
        fd = self.p("follow_distance")
        moving = speed >= self.p("moving_speed")
        out = {"distance": round(dist, 2)}

        if dist > fd + self.p("close_margin") or (moving and dist > fd):
            out["mode"] = "to_last_seen" if lost else "following"
            la = 0.0 if lost else self.p("lookahead")
            goal = self._drive_goal((hx, hy), (hx + hvx * la, hy + hvy * la), (rx, ry), fd)
        else:
            out["mode"] = "facing"
            goal = self._face_goal((hx, hy), robot)
        if goal is not None:
            self._send(goal)
        return out

    def _drive_goal(self, host, target, robot, fd):
        """`fd` short of `target` on the line from the robot, at least `min_distance` from
        the host's current position, facing the target."""
        dx, dy = target[0] - robot[0], target[1] - robot[1]
        n = max(math.hypot(dx, dy), 1e-6)
        gx, gy = target[0] - dx / n * fd, target[1] - dy / n * fd
        hd = math.hypot(gx - host[0], gy - host[1])
        md = self.p("min_distance")
        if hd < md:  # pull the goal back towards the robot, away from the host
            ux, uy = robot[0] - host[0], robot[1] - host[1]
            un = max(math.hypot(ux, uy), 1e-6)
            gx, gy = host[0] + ux / un * md, host[1] + uy / un * md
        return gx, gy, math.atan2(dy, dx), "drive"

    def _face_goal(self, host, robot):
        rx, ry, ryaw = robot
        yaw = math.atan2(host[1] - ry, host[0] - rx)
        if abs(_wrap(yaw - ryaw)) > self.p("face_tolerance"):
            return rx, ry, yaw, "turn"
        if self.last_goal is not None and self.last_goal[3] == "drive":
            self._cancel_goal()  # close enough: stop driving
        return None

    # ------------------------------------------------------------------ nav2

    def _send(self, goal):
        now = time.monotonic()
        if now - self.last_goal_time < 1.0 / self.p("goal_rate"):
            return
        if self.last_goal is not None and self.nav_state in ("active", "pending"):
            moved = math.hypot(goal[0] - self.last_goal[0], goal[1] - self.last_goal[1])
            turned = abs(_wrap(goal[2] - self.last_goal[2]))
            if goal[3] == self.last_goal[3] and moved < self.p("goal_min_move") and (
                    goal[3] == "drive" or turned < self.p("goal_min_turn")):
                return
        ps = self._pose(self.frame, goal)
        self.goal_pub.publish(ps)
        nav_pose = self._to_planner_frame(ps)
        if nav_pose is None:
            return
        self.get_logger().info(
            f"goal ({goal[3]}): ({goal[0]:.2f}, {goal[1]:.2f}, {math.degrees(goal[2]):.0f} deg) "
            f"in {self.frame}, ({nav_pose.pose.position.x:.2f}, {nav_pose.pose.position.y:.2f}) "
            f"in {nav_pose.header.frame_id}", throttle_duration_sec=2.0)
        fut = self.nav.send_goal_async(NavigateToPose.Goal(pose=nav_pose))
        fut.add_done_callback(self._goal_response)
        self.last_goal = goal
        self.last_goal_time = now
        self.nav_state = "pending"

    def _pose(self, frame, goal):
        ps = PoseStamped()
        ps.header.frame_id = frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = goal[0], goal[1]
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(goal[2] / 2), math.cos(goal[2] / 2)
        return ps

    def _to_planner_frame(self, ps):
        """Converted here with the latest transform: Nav2 would convert at the goal's stamp,
        which fails when AMCL publishes map->odom only sparsely ("extrapolation into the
        past") while the robot stands still."""
        target = self.p("planner_frame")
        if ps.header.frame_id == target:
            return ps
        t = self._lookup(target, ps.header.frame_id)
        if t is None:
            return None
        yaw = _yaw(t.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        x, y = ps.pose.position.x, ps.pose.position.y
        return self._pose(target, (t.translation.x + c * x - s * y,
                                   t.translation.y + s * x + c * y,
                                   _yaw(ps.pose.orientation) + yaw))

    def _goal_response(self, fut):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.nav_state = "rejected"
            self.last_goal = None
            self.get_logger().warn("Nav2 rejected the goal", throttle_duration_sec=2.0)
            return
        if not self.active:  # stopped while the goal was on its way
            gh.cancel_goal_async()
            return
        self.goal_handle = gh
        self.nav_state = "active"
        gh.get_result_async().add_done_callback(lambda f, gh=gh: self._goal_done(f, gh))

    def _goal_done(self, fut, gh):
        if gh is not self.goal_handle:
            return  # result of a pre-empted goal
        status = fut.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.nav_state = "reached"
        elif status == GoalStatus.STATUS_ABORTED:
            self.nav_state = "aborted"
            self.last_goal = None  # allow an immediate retry
            self.get_logger().warn("Nav2 aborted the goal", throttle_duration_sec=2.0)
        else:
            self.nav_state = "idle"


def main():
    # Handle Ctrl-C ourselves: rclpy's handler would invalidate the context before we
    # could cancel the goal and restore Nav2's speed limits.
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = FollowManagerNode()
    ex = rclpy.executors.MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    ex.remove_node(node)
    node.shutting_down = True
    try:
        node._stop()
        if node.goal_handle is not None:
            rclpy.spin_once(node, timeout_sec=0.5)
    except Exception as e:  # never block shutdown
        print(f"follow_manager: cleanup failed: {e}")
    node.destroy_node()
    rclpy.try_shutdown()
