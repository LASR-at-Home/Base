"""
Direct velocity controller for person following (replaces Nav2 in `follow_person`).

Runs concurrently with the tracker and talks to it only through the blackboard:

    reads   host_state            {"pos": (x, y), "vel": (vx, vy), "stamp": monotonic s}
            other_people          [{"pos": (x, y), "vel": (vx, vy), "stamp": ...}, ...]
            stop_robot_requested  hold position (the robot may still turn to face the host)
            cancel_nav            stop and exit with "canceled"

All positions are in `map`. Each control tick (20 Hz):

    1. Breadcrumbs: every new host position is appended to a trail. The robot steers at a
       point `lookahead` metres along that trail rather than straight at the host, so it
       takes roughly the path the host walked instead of cutting corners through
       walls/tables.
    2. Gap control: forward speed is proportional to (predicted distance to host -
       safe_distance), where the prediction is the host's position `predict_horizon`
       seconds ahead from their estimated velocity. A host walking away speeds the robot
       up early; a host stopping/coming back slows it down before the gap closes.
    3. Other people: each one is projected forward along their velocity; speed is scaled
       down (to zero) if the robot's own short-term path would come too close to them.
    4. Laser failsafe: anything in the corridor straight ahead limits the speed and stops
       the robot at `front_stop`; walls closer than `wall_margin` to either side slow the
       robot and push its heading away from them. With no recent laser scan the robot
       does not drive forwards at all.
"""

import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data

import yasmin
from yasmin import State, Blackboard
import yasmin_ros

import tf2_ros

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _ramp(value: float, zero_at: float, one_at: float) -> float:
    """0 at `zero_at`, 1 at `one_at`, linear in between."""
    if one_at <= zero_at:
        return 1.0 if value >= one_at else 0.0
    return _clamp((value - zero_at) / (one_at - zero_at), 0.0, 1.0)


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class FollowVelocityController(State):
    """
    Args:
        safe_distance: gap (m, centre to centre) to keep to the host.
        max_linear / max_angular: velocity limits (m/s, rad/s).
        cmd_vel_topic / scan_topic: base velocity command and laser scan topics.
        robot_radius: base radius (m). laser_offset: laser x position in base_footprint.
        front_stop / front_slow: free space ahead of the base (m) at which the robot
            stops / starts slowing down.
        wall_margin: side clearance (m) below which the robot steers away from a wall.
        person_stop / person_slow: predicted distance to another person (m) at which the
            robot stops / starts slowing down.
    """

    def __init__(
        self,
        safe_distance: float = 1.0,
        max_linear: float = 0.4,
        max_angular: float = 0.4,
        cmd_vel_topic: str = "/cmd_vel",
        scan_topic: str = "/scan_raw",
        rate: float = 20.0,
        # trail following
        crumb_spacing: float = 0.15,
        lookahead: float = 0.6,
        # gap control
        gap_gain: float = 0.7,
        predict_horizon: float = 0.8,
        heading_gain: float = 1.8,
        linear_accel: float = 0.4,
        linear_decel: float = 1.0,
        angular_accel: float = 1.0,
        # laser failsafe
        robot_radius: float = 0.27,
        laser_offset: float = 0.2,
        front_stop: float = 0.2,
        front_slow: float = 0.8,
        wall_margin: float = 0.3,
        wall_gain: float = 0.8,
        scan_timeout: float = 0.5,
        # other people
        person_stop: float = 0.5,
        person_slow: float = 1.0,
        person_horizon: float = 2.0,
    ):
        super().__init__(outcomes=["succeeded", "canceled", "aborted"])
        self.add_input_key("host_state")
        self.add_input_key("other_people")
        self.add_input_key("cancel_nav")
        self.add_input_key("stop_robot_requested")

        self.safe_distance = safe_distance
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.period = 1.0 / rate
        self.crumb_spacing = crumb_spacing
        self.lookahead = lookahead
        self.gap_gain = gap_gain
        self.predict_horizon = predict_horizon
        self.heading_gain = heading_gain
        self.linear_accel = linear_accel
        self.linear_decel = linear_decel
        self.angular_accel = angular_accel
        self.robot_radius = robot_radius
        self.laser_offset = laser_offset
        self.front_stop = front_stop
        self.front_slow = front_slow
        self.wall_margin = wall_margin
        self.wall_gain = wall_gain
        self.scan_timeout = scan_timeout
        self.person_stop = person_stop
        self.person_slow = person_slow
        self.person_horizon = person_horizon
        self.scan_topic = scan_topic

        self.node = yasmin_ros.logger_node
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        self.cmd_pub = self.node.create_publisher(Twist, cmd_vel_topic, 10)
        self.trail_pub = self.node.create_publisher(
            Path, "/person_follow/debug/trail", 1
        )
        self._scan = None
        self._scan_time = 0.0
        self.scan_sub = self.node.create_subscription(
            LaserScan, scan_topic, self._scan_cb, qos_profile_sensor_data
        )

        self._last_log = {}

    # ---- inputs ----

    def _scan_cb(self, msg: LaserScan):
        self._scan = msg
        self._scan_time = time.monotonic()

    def _robot_pose(self):
        """(x, y, yaw) of base_footprint in map, or None."""
        try:
            t = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            ).transform
            q = t.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            return t.translation.x, t.translation.y, yaw
        except Exception:
            return None

    def _log(self, key: str, msg: str, period: float = 2.0):
        now = time.monotonic()
        if now - self._last_log.get(key, 0.0) >= period:
            self._last_log[key] = now
            yasmin.YASMIN_LOG_WARN(f"[FollowVelocity] {msg}")

    # ---- laser failsafe ----

    def _laser_limits(self):
        """
        Returns (speed_factor in [0, 1], angular push in rad/s) from the latest scan.
        (0, 0) if there is no recent scan: never drive forwards blind.
        """
        scan = self._scan
        if scan is None or time.monotonic() - self._scan_time > self.scan_timeout:
            self._log("scan", f"No recent laser scan on {self.scan_topic}: not driving")
            return 0.0, 0.0

        ranges = np.asarray(scan.ranges, dtype=float)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = np.isfinite(ranges) & (ranges > max(scan.range_min, 0.05))
        if not np.any(valid):
            return 1.0, 0.0
        # Points in base_footprint (laser assumed aligned with the base, offset in x).
        x = ranges[valid] * np.cos(angles[valid]) + self.laser_offset
        y = ranges[valid] * np.sin(angles[valid])

        # Straight ahead: anything in the corridor the base would sweep through.
        ahead = (x > 0.0) & (np.abs(y) < self.robot_radius + 0.05)
        front_clear = (
            float(np.min(x[ahead])) - self.robot_radius if np.any(ahead) else math.inf
        )
        factor = _ramp(front_clear, self.front_stop, self.front_slow)
        if factor <= 0.0:
            self._log("front", f"Obstacle {front_clear:.2f}m ahead: stopped")

        # Sides: walls alongside / just ahead of the base.
        beside = (x > -0.1) & (x < self.robot_radius + 0.4)
        push = 0.0
        tightest = math.inf
        for sign in (1.0, -1.0):  # left, right
            side = beside & (sign * y > 0.0)
            if not np.any(side):
                continue
            clear = float(np.min(np.abs(y[side]))) - self.robot_radius
            tightest = min(tightest, clear)
            if clear < self.wall_margin:
                # Left wall -> turn right (negative), right wall -> turn left.
                push -= sign * self.wall_gain * (1.0 - clear / self.wall_margin)
        if tightest < self.wall_margin:
            # Slow down next to walls, but keep moving so doorways are passable.
            factor *= 0.4 + 0.6 * _clamp(tightest / self.wall_margin, 0.0, 1.0)

        return factor, push

    # ---- other people ----

    def _people_factor(self, blackboard, pose, speed: float, now: float) -> float:
        """Speed factor from the predicted closest approach to every other person."""
        try:
            people = blackboard["other_people"] or []
        except Exception:
            return 1.0

        rx, ry, yaw = pose
        hx, hy = math.cos(yaw), math.sin(yaw)
        factor = 1.0
        for person in people:
            if now - person["stamp"] > 1.0:
                continue
            px, py = person["pos"]
            vx, vy = person["vel"]
            closest = math.inf
            for step in range(int(self.person_horizon / 0.25) + 1):
                t = step * 0.25
                dx = (px + vx * t) - (rx + hx * speed * t)
                dy = (py + vy * t) - (ry + hy * speed * t)
                # Only people the robot would be driving towards matter.
                if dx * hx + dy * hy < 0.0:
                    continue
                closest = min(closest, math.hypot(dx, dy))
            factor = min(factor, _ramp(closest, self.person_stop, self.person_slow))
        if factor <= 0.0:
            self._log("person", "Someone is crossing my path: waiting")
        return factor

    # ---- trail ----

    def _target_on_trail(self, trail, pose, fallback):
        """Drops crumbs already passed and returns the point `lookahead` along the rest."""
        rx, ry, _ = pose
        if trail:
            dists = [math.hypot(x - rx, y - ry) for x, y in trail]
            for _ in range(dists.index(min(dists))):
                trail.popleft()
            for x, y in trail:
                if math.hypot(x - rx, y - ry) >= self.lookahead:
                    return x, y
        return fallback

    def _publish_trail(self, trail):
        path = Path()
        path.header.frame_id = "map"
        for x, y in trail:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.trail_pub.publish(path)

    # ---- main loop ----

    def _command(self, blackboard, trail, pose, host, now):
        """Desired (linear, angular) before acceleration limiting."""
        rx, ry, yaw = pose
        hx, hy = host["pos"]
        visible = now - host["stamp"] < 0.6
        holding = bool(blackboard["stop_robot_requested"])

        if holding:
            # Hold position, but keep facing a visible host so they stay in view.
            if not visible:
                return 0.0, 0.0
            error = _wrap(math.atan2(hy - ry, hx - rx) - yaw)
            if abs(error) < 0.15:
                return 0.0, 0.0
            return 0.0, _clamp(
                self.heading_gain * error, -self.max_angular, self.max_angular
            )

        # Gap control on the host's projected position.
        vx, vy = host["vel"] if visible else (0.0, 0.0)
        if math.hypot(vx, vy) < 0.2:  # detection noise
            vx, vy = 0.0, 0.0
        predicted_gap = math.hypot(
            hx + vx * self.predict_horizon - rx, hy + vy * self.predict_horizon - ry
        )
        error = predicted_gap - self.safe_distance
        linear = _clamp(self.gap_gain * error, 0.0, self.max_linear)
        if error < 0.05:
            linear = 0.0

        # Steer along the host's trail.
        tx, ty = self._target_on_trail(trail, pose, (hx, hy))
        heading_error = _wrap(math.atan2(ty - ry, tx - rx) - yaw)
        angular = self.heading_gain * heading_error
        # Turn (nearly) on the spot when facing the wrong way.
        linear *= 1.0 - _ramp(abs(heading_error), 0.3, 1.0)

        laser_factor, wall_push = self._laser_limits()
        linear *= laser_factor
        if linear > 0.0:
            angular += wall_push
            linear *= self._people_factor(blackboard, pose, linear, now)

        return linear, _clamp(angular, -self.max_angular, self.max_angular)

    def execute(self, blackboard: Blackboard):
        for key, default in (
            ("host_state", None),
            ("other_people", []),
            ("cancel_nav", False),
            ("stop_robot_requested", False),
        ):
            if key not in blackboard.keys():
                blackboard[key] = default

        trail = deque(maxlen=400)
        last_stamp = None
        linear = 0.0
        angular = 0.0
        tick = 0

        while not self.is_canceled() and rclpy.ok():
            start = time.monotonic()
            if blackboard["cancel_nav"]:
                self.cancel_state()
                break

            pose = self._robot_pose()
            host = blackboard["host_state"]
            want_linear, want_angular = 0.0, 0.0

            if pose is None:
                self._log("tf", "No map->base_footprint transform: not driving")
            elif host is not None:
                if host["stamp"] != last_stamp:
                    last_stamp = host["stamp"]
                    if not trail or (
                        math.hypot(
                            host["pos"][0] - trail[-1][0], host["pos"][1] - trail[-1][1]
                        )
                        >= self.crumb_spacing
                    ):
                        trail.append(tuple(host["pos"]))
                want_linear, want_angular = self._command(
                    blackboard, trail, pose, host, start
                )
            else:
                trail.clear()
                last_stamp = None

            # Acceleration limits (braking is allowed to be sharper than speeding up).
            lin_step = (
                self.linear_accel if want_linear > linear else self.linear_decel
            ) * self.period
            linear += _clamp(want_linear - linear, -lin_step, lin_step)
            ang_step = self.angular_accel * self.period
            angular += _clamp(want_angular - angular, -ang_step, ang_step)

            cmd = Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            self.cmd_pub.publish(cmd)

            tick += 1
            if tick % 10 == 0:
                self._publish_trail(trail)

            time.sleep(max(0.0, self.period - (time.monotonic() - start)))

        for _ in range(5):
            self.cmd_pub.publish(Twist())
            time.sleep(0.02)

        return "canceled" if self.is_canceled() else "succeeded"
