"""
follow_manager_node: drives the robot after the host using Nav2 only (no direct cmd_vel).

Reads the host estimate from person_tracker (~/host_state) and streams NavigateToPose
goals through the robot's normal bt_navigator (a new goal pre-empts the running one
without stopping, so nothing custom has to be installed on the robot):

  * goal = a point on the host's own walked trail, `follow_distance` (path length) behind
    their predicted position, facing them. The trail is free space by construction, so the
    goal never lands inside the person's costmap footprint, and around corners the robot
    takes the host's route instead of cutting through walls.
  * when the robot is already within `follow_distance` it only turns to face the host
    (goal = current position, new yaw), keeping them in the camera.
  * goals are re-sent at most `goal_rate` Hz and only when they moved meaningfully.
  * while following, Nav2's speed limits are raised (dynamic parameters) and restored on stop.

Services:  ~/start, ~/stop  (std_srvs/Trigger)
Topic:     ~/status         (std_msgs/String JSON: mode, host status, distance, nav state)
"""

import json
import math
import time
from collections import deque

import rclpy
import rclpy.signals
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import ParameterType

import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import BackUp, FollowPath, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
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
        # odom, not map: following is relative to the host, so the trail, the robot pose and
        # FollowPath stay correct even if AMCL jumps (NavigateToPose goals are converted by
        # Nav2 with the current localisation, which keeps them right relative to the robot)
        d("global_frame", "odom")
        d("robot_frame", "base_footprint")
        d("odom_frame", "odom")
        d("planner_frame", "map")  # Nav2's global frame, for NavigateToPose goals
        d("follow_distance", 1.0)  # m of trail between robot goal and host
        d("close_margin", 0.5)  # slow host within follow_distance + this: only turn
        # (must exceed Nav2's xy goal tolerance, or the robot never counts as settled)
        d("min_distance", 0.9)  # never drive when closer than this to the host
        d("moving_speed", 0.25)  # host speed (m/s) above which they count as walking
        d("lag_time", 2.0)  # s, controller lag behind a moving goal (see _tick)
        d("min_back", 0.3)  # m, closest goal behind a walking host
        d("face_tolerance", 0.35)  # rad heading error before turning on the spot
        # a close host at a wide bearing (e.g. walking past the robot) would leave the
        # head's range: turn the base towards them whatever the mode
        d("turn_bearing", 1.2)  # rad
        d("turn_distance", 2.5)  # m
        d("lookahead", 0.6)  # s of host motion to predict
        d("crumb_spacing", 0.15)
        d("goal_rate", 2.0)  # max goal updates per second
        d("goal_min_move", 0.25)  # m change needed to re-send a goal
        d("goal_min_turn", 0.3)  # rad change needed to re-send a turn-only goal
        d("rate", 10.0)
        # When the robot is on the host's trail, the trail itself is sent to the
        # controller (Nav2 FollowPath): the robot takes exactly the host's route, e.g.
        # through the middle of a doorway. Otherwise NavigateToPose plans a path.
        d("use_trail_path", True)
        d("on_trail_distance", 0.6)  # m, robot must be this close to the trail
        # stuck watchdog: no progress while it should be driving -> clear costmaps and
        # back up (Nav2's own recovery never triggers because goals keep being replaced)
        d("stuck_time", 4.0)  # s
        d("stuck_distance", 0.15)  # m
        d("backup_distance", 0.3)  # m
        # speed limits while following (restored afterwards); empty name = skip
        d("follow_max_speed", 0.7)
        d("controller_node", "/controller_server")
        d("controller_speed_params", ["FollowPath.max_vel_x", "FollowPath.max_speed_xy"])
        d("smoother_node", "/velocity_smoother")
        d("smoother_speed_param", "max_velocity")

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.global_frame = p("global_frame")
        self.robot_frame = p("robot_frame")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose",
                                callback_group=ReentrantCallbackGroup())
        self.follow_path = ActionClient(self, FollowPath, "/follow_path",
                                        callback_group=ReentrantCallbackGroup())
        self.goal_seg = None  # trail index just after the current drive goal
        self.goal_kind = None  # "path" (FollowPath) or "navigate" (NavigateToPose)

        self.host = None  # latest host_state dict
        self.create_subscription(String, p("host_state_topic"), self._host_cb, 10)
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "~/goal", 10)
        self.trail_pub = self.create_publisher(Path, "~/trail", 1)
        # services block on parameter calls: they and those clients must not share the
        # default (mutually exclusive) group, or the replies could never be processed
        self.rg = ReentrantCallbackGroup()
        self.create_service(Trigger, "~/start", self._start_srv, callback_group=self.rg)
        self.create_service(Trigger, "~/stop", self._stop_srv, callback_group=self.rg)

        self.active = False
        self.trail = deque(maxlen=500)  # host positions in global frame
        self.goal_handle = None
        self.last_goal = None  # (x, y, yaw, kind)
        self.last_goal_time = 0.0
        self.nav_state = "idle"
        self.nav_failures = 0
        self.saved_speed = {}
        self.recovering = False
        self.shutting_down = False
        self.progress = None  # (x, y, time) of the last progress
        self.backup = ActionClient(self, BackUp, "/backup", callback_group=ReentrantCallbackGroup())
        self.clear_clients = [
            self.create_client(ClearEntireCostmap, n, callback_group=ReentrantCallbackGroup())
            for n in ("/global_costmap/clear_entirely_global_costmap",
                      "/local_costmap/clear_entirely_local_costmap")]
        self.create_timer(1.0 / p("rate"), self._tick)

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
        """(x, y, yaw) of the robot in the global frame.

        Composed from the latest global<-odom and the latest odom<-robot separately: AMCL
        publishes map->odom only occasionally while the robot stands still, and a single
        chained lookup then fails with "extrapolation into the past" (the robot would never
        start moving, so AMCL would never publish again)."""
        odom = self.get_parameter("odom_frame").value
        a = self._lookup(self.global_frame, odom)
        b = self._lookup(odom, self.robot_frame)
        if a is None or b is None:
            return None
        ay = _yaw(a.rotation)
        c, s = math.cos(ay), math.sin(ay)
        x = a.translation.x + c * b.translation.x - s * b.translation.y
        y = a.translation.y + s * b.translation.x + c * b.translation.y
        return x, y, _wrap(ay + _yaw(b.rotation))

    def _to_global(self, x, y, frame):
        if frame == self.global_frame:
            return x, y
        t = self._lookup(self.global_frame, frame)
        if t is None:
            return None
        yaw = _yaw(t.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        return t.translation.x + c * x - s * y, t.translation.y + s * x + c * y

    def _vel_to_global(self, vx, vy, frame):
        if frame == self.global_frame:
            return vx, vy
        t = self._lookup(self.global_frame, frame)
        if t is None:
            return vx, vy
        yaw = _yaw(t.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        return c * vx - s * vy, s * vx + c * vy

    # ------------------------------------------------------------------ speed params

    def _param_call(self, node, srv_type, req, timeout=2.0):
        cli = self.create_client(srv_type, f"{node}/{'get' if srv_type is GetParameters else 'set'}_parameters",
                                 callback_group=self.rg)
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
        v = float(self.get_parameter("follow_max_speed").value)
        if v <= 0:
            return
        ctrl = self.get_parameter("controller_node").value
        names = list(self.get_parameter("controller_speed_params").value)
        if ctrl and names:
            res = self._param_call(ctrl, GetParameters, GetParameters.Request(names=names))
            if res is not None:
                self.saved_speed[ctrl] = [
                    Parameter(n, value=pv.double_value) for n, pv in zip(names, res.values)
                    if pv.type == ParameterType.PARAMETER_DOUBLE]
                self._param_call(ctrl, SetParameters, SetParameters.Request(parameters=[
                    Parameter(n, value=v).to_parameter_msg() for n in names]))
        sm = self.get_parameter("smoother_node").value
        sp = self.get_parameter("smoother_speed_param").value
        if sm and sp:
            res = self._param_call(sm, GetParameters, GetParameters.Request(names=[sp, "min_velocity"]))
            if res is not None and res.values and res.values[0].type == ParameterType.PARAMETER_DOUBLE_ARRAY:
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
        self.trail.clear()
        self.last_goal = None
        self.nav_failures = 0
        self.recovering = False
        self.progress = None
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
        status = {"active": self.active, "nav": self.nav_state, "host": None,
                  "distance": None, "mode": "idle"}
        host = self.host
        robot = self._robot_pose()
        if self.active and robot is None:
            self.get_logger().warn("no robot pose in the global frame", throttle_duration_sec=2.0)
        if host is not None:
            status["host"] = host.get("status")
        if not self.active or host is None or robot is None or host.get("x") is None:
            self.status_pub.publish(String(data=json.dumps(status)))
            return

        rx, ry, ryaw = robot
        if self.recovering:
            status["mode"] = "recovering"
            self.status_pub.publish(String(data=json.dumps(status)))
            return
        hp = self._to_global(host["x"], host["y"], host["frame"])
        if hp is None:
            return
        hvx, hvy = self._vel_to_global(host["vx"], host["vy"], host["frame"])
        tracking = host["status"] == "tracking"
        lost = host["status"] == "lost"

        # trail of where the host actually walked (only from real observations)
        if tracking and (not self.trail or math.hypot(hp[0] - self.trail[-1][0], hp[1] - self.trail[-1][1])
                         >= self.get_parameter("crumb_spacing").value):
            self.trail.append(hp)

        speed = math.hypot(hvx, hvy) if tracking else 0.0
        la = self.get_parameter("lookahead").value if tracking else 0.0
        if lost:  # head for where they were going (as the tracker's re-identification does)
            la = min(host.get("last_seen_age") or 0.0, 2.0)
        target = (hp[0] + hvx * la, hp[1] + hvy * la)
        host_dist = math.hypot(hp[0] - rx, hp[1] - ry)
        status["distance"] = round(host_dist, 2)
        fd = self.get_parameter("follow_distance").value
        margin = self.get_parameter("close_margin").value
        moving = speed >= self.get_parameter("moving_speed").value

        bearing = _wrap(math.atan2(target[1] - ry, target[0] - rx) - ryaw)
        if (not lost and abs(bearing) > self.get_parameter("turn_bearing").value
                and host_dist < self.get_parameter("turn_distance").value):
            status["mode"] = "turning"
            goal = (rx, ry, math.atan2(target[1] - ry, target[0] - rx), "turn")
        elif lost and host_dist > fd + margin:
            # drive towards where the host was last seen (end of trail), stopping a
            # follow_distance short (they may still be standing there, unseen); the state
            # machine decides on recovery behaviour
            status["mode"] = "to_last_seen"
            goal = self._trail_goal(target, (rx, ry), fd)
        elif lost:
            status["mode"] = "facing"
            goal = self._face_goal(hp, rx, ry, ryaw)
        elif host_dist < self.get_parameter("min_distance").value or (
                not moving and host_dist <= fd + margin):
            # close enough (or too close): stop and just face the host
            status["mode"] = "facing"
            goal = self._face_goal(hp, rx, ry, ryaw)
        else:
            status["mode"] = "following"
            # Nav2's controller settles about speed * lag_time short of a moving goal,
            # so the goal is moved closer to a walking host to keep ~follow_distance.
            # Only while actually lagging behind: near the host (e.g. as they slow down)
            # the goal stays a full follow_distance back so the robot can't overshoot.
            back = fd
            if moving and host_dist > fd + margin:
                back = max(self.get_parameter("min_back").value,
                           fd - self.get_parameter("lag_time").value * speed)
            goal = self._trail_goal(target, (rx, ry), back)

        if goal is not None:
            self._maybe_send(goal, (rx, ry, ryaw))
        self._check_progress(rx, ry, status["mode"])
        # settled: close to a slow host and no longer driving (safe to talk to them)
        status["settled"] = status["mode"] == "facing" and self.nav_state != "active"
        status["nav"] = self.nav_state
        self.status_pub.publish(String(data=json.dumps(status)))

    def _check_progress(self, rx, ry, mode):
        now = self.get_clock().now().nanoseconds * 1e-9
        driving = (mode in ("following", "to_last_seen") and self.last_goal is not None
                   and self.last_goal[3] == "drive" and self.nav_state in ("active", "pending")
                   and math.hypot(self.last_goal[0] - rx, self.last_goal[1] - ry) > 0.5)
        if not driving or self.progress is None:
            self.progress = (rx, ry, now)
            return
        if math.hypot(rx - self.progress[0], ry - self.progress[1]) > self.get_parameter("stuck_distance").value:
            self.progress = (rx, ry, now)
        elif now - self.progress[2] > self.get_parameter("stuck_time").value:
            self.get_logger().warn("no progress: clearing costmaps and backing up")
            self._recover()
            self.progress = None

    def _recover(self):
        self.recovering = True
        self._cancel_goal()
        for cli in self.clear_clients:
            if cli.service_is_ready():
                cli.call_async(ClearEntireCostmap.Request())
        if not self.backup.server_is_ready():
            self.recovering = False
            return
        goal = BackUp.Goal()
        goal.target = Point(x=float(self.get_parameter("backup_distance").value))
        goal.speed = 0.15
        goal.time_allowance = Duration(seconds=6.0).to_msg()
        fut = self.backup.send_goal_async(goal)

        def accepted(f):
            gh = f.result()
            if gh is None or not gh.accepted:
                self.recovering = False
                return
            gh.get_result_async().add_done_callback(lambda _: setattr(self, "recovering", False))
        fut.add_done_callback(accepted)

    def _face_goal(self, hp, rx, ry, ryaw):
        yaw = math.atan2(hp[1] - ry, hp[0] - rx)
        if abs(_wrap(yaw - ryaw)) < self.get_parameter("face_tolerance").value:
            if self.last_goal is not None and self.last_goal[3] == "drive":
                self._cancel_goal()  # stop driving
            elif self.last_goal is not None and self.nav_state != "active":
                self.last_goal = None
            return None
        return (rx, ry, yaw, "turn")

    def _trail_goal(self, target, robot, back):
        """Point `back` metres of path behind `target` along the host trail, facing target."""
        pts = list(self.trail) + [target]
        self.goal_seg = None
        acc = 0.0
        gx, gy = target
        heading = None  # direction of travel along the trail at the goal
        found = False
        for i in range(len(pts) - 1, 0, -1):
            (ax, ay), (bx, by) = pts[i - 1], pts[i]
            seg = math.hypot(bx - ax, by - ay)
            if acc + seg >= back:
                f = (back - acc) / max(seg, 1e-6)
                gx, gy = bx + (ax - bx) * f, by + (ay - by) * f
                found = True
                self.goal_seg = i  # goal lies between pts[i-1] and pts[i]
                # smooth heading over ~0.6 m of trail before the goal
                j = i - 1
                while j > 0 and math.hypot(gx - pts[j][0], gy - pts[j][1]) < 0.6:
                    j -= 1
                if math.hypot(gx - pts[j][0], gy - pts[j][1]) > 0.1:
                    heading = math.atan2(gy - pts[j][1], gx - pts[j][0])
                break
            acc += seg
        if not found:
            # short trail: back off from the target towards the robot
            dx, dy = robot[0] - target[0], robot[1] - target[1]
            n = math.hypot(dx, dy)
            if n > 1e-3:
                k = min(back, n) / n
                gx, gy = target[0] + dx * k, target[1] + dy * k
        # never drive to a point that is behind the robot relative to the host
        if math.hypot(target[0] - robot[0], target[1] - robot[1]) < math.hypot(target[0] - gx, target[1] - gy):
            gx, gy = robot
        # Face along the trail (the way the robot is driving), not at the host: a goal
        # yaw that keeps changing makes the controller weave.
        if heading is None:
            heading = math.atan2(target[1] - gy, target[0] - gx)
        return gx, gy, heading, "drive"

    def _trail_path(self, goal, robot):
        """Path from the robot along the host trail to `goal`, or None if the robot is
        not on the trail (then the planner has to find a way)."""
        if not self.get_parameter("use_trail_path").value or self.goal_seg is None:
            return None
        pts = list(self.trail)[: self.goal_seg]
        if not pts:
            return None
        dists = [math.hypot(x - robot[0], y - robot[1]) for x, y in pts]
        k = min(range(len(dists)), key=dists.__getitem__)
        if dists[k] > self.get_parameter("on_trail_distance").value:
            return None
        way = [robot[:2]] + pts[k + 1:] + [goal[:2]]
        path = Path()
        path.header.frame_id = self.global_frame  # zero stamp: latest transform
        for (ax, ay), (bx, by) in zip(way, way[1:]):
            seg = math.hypot(bx - ax, by - ay)
            yaw = math.atan2(by - ay, bx - ax)
            n = max(1, int(seg / 0.1))
            for j in range(n):
                ps = PoseStamped()
                ps.header = path.header
                ps.pose.position.x = ax + (bx - ax) * j / n
                ps.pose.position.y = ay + (by - ay) * j / n
                ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                path.poses.append(ps)
        end = PoseStamped()
        end.header = path.header
        end.pose.position.x, end.pose.position.y = goal[0], goal[1]
        end.pose.orientation.z, end.pose.orientation.w = math.sin(goal[2] / 2), math.cos(goal[2] / 2)
        path.poses.append(end)
        return path if len(path.poses) >= 2 else None

    def _maybe_send(self, goal, robot=None):
        now = time.monotonic()
        if now - self.last_goal_time < 1.0 / self.get_parameter("goal_rate").value:
            return
        if self.last_goal is not None and self.nav_state in ("active", "pending"):
            moved = math.hypot(goal[0] - self.last_goal[0], goal[1] - self.last_goal[1])
            turned = abs(_wrap(goal[2] - self.last_goal[2]))
            if moved < self.get_parameter("goal_min_move").value and (
                    goal[3] == "drive" or turned < self.get_parameter("goal_min_turn").value):
                return
        ps = PoseStamped()
        ps.header.frame_id = self.global_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = goal[0], goal[1]
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(goal[2] / 2), math.cos(goal[2] / 2)
        self.goal_pub.publish(ps)
        self.get_logger().info(
            f"goal ({goal[3]}): ({goal[0]:.2f}, {goal[1]:.2f}, {math.degrees(goal[2]):.0f} deg)",
            throttle_duration_sec=2.0)
        path = self._trail_path(goal, robot) if goal[3] == "drive" and robot is not None else None
        if path is not None and self.follow_path.server_is_ready():
            if self.goal_handle is not None and self.goal_kind == "navigate":
                self.goal_handle.cancel_goal_async()  # don't let the BT fight over the controller
                self.goal_handle = None
            fut = self.follow_path.send_goal_async(FollowPath.Goal(path=path))
            self.goal_kind = "path"
        else:
            if self.goal_handle is not None and self.goal_kind == "path":
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
            nav_pose = self._to_planner_frame(ps)
            if nav_pose is None:
                return
            fut = self.nav.send_goal_async(NavigateToPose.Goal(pose=nav_pose))
            self.goal_kind = "navigate"
        fut.add_done_callback(self._goal_response)
        self.last_goal = goal
        self.last_goal_time = now
        self.nav_state = "pending"
        self._publish_trail()

    def _to_planner_frame(self, ps):
        """NavigateToPose goals go to Nav2 in its planner frame (map), converted with the
        latest transform: Nav2 converts at the goal's stamp, which fails when AMCL's
        map->odom is published only sparsely ("extrapolation into the past")."""
        target = self.get_parameter("planner_frame").value
        if ps.header.frame_id == target:
            return ps
        t = self._lookup(target, ps.header.frame_id)
        if t is None:
            return None
        yaw = _yaw(t.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        x, y = ps.pose.position.x, ps.pose.position.y
        gyaw = _yaw(ps.pose.orientation) + yaw
        out = PoseStamped()
        out.header.frame_id = target
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose.position.x = t.translation.x + c * x - s * y
        out.pose.position.y = t.translation.y + s * x + c * y
        out.pose.orientation.z, out.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
        return out

    def _goal_response(self, fut):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.nav_state = "rejected"
            self.get_logger().warn("Nav2 rejected the goal", throttle_duration_sec=2.0)
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
            self.nav_failures = 0
        elif status == GoalStatus.STATUS_ABORTED:
            self.nav_state = "aborted"
            self.get_logger().warn("Nav2 aborted the goal", throttle_duration_sec=2.0)
            self.nav_failures += 1
            self.last_goal = None  # allow an immediate retry
        else:
            self.nav_state = "idle"

    def _publish_trail(self):
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.trail:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.trail_pub.publish(path)

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
