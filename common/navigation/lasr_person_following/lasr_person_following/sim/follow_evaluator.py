"""
Scores a person-following run in simulation (sim testing only).

Ground truth comes from Gazebo, all in the `world` frame (= robot odom at start):
    /ground_truth_odom            robot (TIAGo p3d plugin)
    /<host>/ground_truth          host  (scripted_person)
    /<distractor>/ground_truth    optional distractor

Optional tracker estimate: `estimate_topic` (geometry_msgs/PointStamped in any frame
that TF can transform to `odom`; odom == world in this sim).

The run ends `settle_time` seconds after the host's scenario reports "done" (or after
`max_duration`). A CSV time series and a JSON summary are written to `out_dir`/`run_name`.
"""

import csv
import json
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import tf2_ros
from tf2_geometry_msgs import do_transform_point


class FollowEvaluator(Node):
    def __init__(self):
        super().__init__("follow_evaluator")
        self.declare_parameter("host", "test_person")
        self.declare_parameter("distractor", "")
        self.declare_parameter("estimate_topic", "")
        self.declare_parameter("out_dir", "/tmp/follow_eval")
        self.declare_parameter("run_name", "run")
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("settle_time", 20.0)
        self.declare_parameter("max_duration", 300.0)
        self.declare_parameter("lost_distance", 3.5)  # robot left behind beyond this
        self.declare_parameter("arrive_distance", 2.0)
        self.declare_parameter("map_from_world", [9.151, -6.340, 2.204])

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.host = p("host")
        self.distractor = p("distractor")
        self.out_dir = os.path.join(p("out_dir"), p("run_name"))
        self.settle_time = p("settle_time")
        self.max_duration = p("max_duration")
        self.lost_distance = p("lost_distance")
        self.arrive_distance = p("arrive_distance")

        self.robot = None
        self.host_pos = None
        self.host_status = "waiting"
        self.distractor_pos = None
        self.estimate = None  # (x, y, wall time)
        self.scan_min = math.inf

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Odometry, "/ground_truth_odom", self._robot_cb, 10)
        self.create_subscription(
            PoseStamped, f"/{self.host}/ground_truth", self._host_cb, 10
        )
        self.create_subscription(String, f"/{self.host}/status", self._status_cb, 10)
        if self.distractor:
            self.create_subscription(
                PoseStamped, f"/{self.distractor}/ground_truth", self._distractor_cb, 10
            )
        if p("estimate_topic"):
            self.create_subscription(
                PointStamped, p("estimate_topic"), self._estimate_cb, 10
            )
        self.create_subscription(
            LaserScan, "/scan_raw", self._scan_cb, qos_profile_sensor_data
        )
        self.amcl = None
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 lambda m: setattr(self, "amcl", (m.pose.pose.position.x, m.pose.pose.position.y)), 10)

        self.rows = []
        self.t0 = None
        self.done_at = None
        self.created = None
        self.finished = False
        self.create_timer(1.0 / p("rate"), self._tick)

    # ---- inputs ----

    def _robot_cb(self, m: Odometry):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.robot = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def _host_cb(self, m: PoseStamped):
        self.host_pos = (m.pose.position.x, m.pose.position.y)

    def _status_cb(self, m: String):
        self.host_status = m.data

    def _distractor_cb(self, m: PoseStamped):
        self.distractor_pos = (m.pose.position.x, m.pose.position.y)

    def _estimate_cb(self, m: PointStamped):
        """Store the estimate in `world`: odom via TF, then world <- odom from GT."""
        try:
            if m.header.frame_id != "odom":
                t = self.tf_buffer.lookup_transform("odom", m.header.frame_id, rclpy.time.Time())
                m = do_transform_point(m, t)
            wx, wy = self._odom_to_world(m.point.x, m.point.y)
            self.estimate = (wx, wy, self._now())
        except Exception as e:
            self.get_logger().warn(f"estimate TF failed: {e}", throttle_duration_sec=5.0)

    def _now(self):
        """Sim time (use_sim_time) so durations are independent of the real-time factor."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom_to_world(self, x, y):
        """world <- odom = GT(world <- base) * inv(TF(odom <- base)). Handles odom drift."""
        t = self.tf_buffer.lookup_transform("odom", "base_footprint", rclpy.time.Time()).transform
        q = t.rotation
        oyaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        gx, gy, gyaw = self.robot
        dyaw = gyaw - oyaw
        # point relative to the robot in odom, rotated into world, added to GT robot pose
        ux, uy = x - t.translation.x, y - t.translation.y
        c, s = math.cos(dyaw), math.sin(dyaw)
        return gx + c * ux - s * uy, gy + s * ux + c * uy

    def _scan_cb(self, m: LaserScan):
        r = np.asarray(m.ranges, dtype=float)
        r = r[np.isfinite(r) & (r > m.range_min)]
        if r.size:
            self.scan_min = float(r.min())

    # ---- loop ----

    def _tick(self):
        if self.finished or self.robot is None or self.host_pos is None:
            return
        now = self._now()
        if self.host_status == "waiting":
            if self.created is None:
                self.created = now
            elif now - self.created > 120.0:
                self.get_logger().error("SUMMARY {\"error\": \"host never started walking\"}")
                self.finished = True
                raise SystemExit
            return
        if self.t0 is None:
            self.t0 = now
            self.get_logger().info("Host started walking: recording")
        t = now - self.t0

        rx, ry, ryaw = self.robot
        hx, hy = self.host_pos
        dist = math.hypot(hx - rx, hy - ry)
        bearing = math.degrees(
            math.atan2(math.sin(math.atan2(hy - ry, hx - rx) - ryaw),
                       math.cos(math.atan2(hy - ry, hx - rx) - ryaw))
        )
        est_err = math.nan
        est_age = math.nan
        if self.estimate is not None:
            est_err = math.hypot(self.estimate[0] - hx, self.estimate[1] - hy)
            est_age = now - self.estimate[2]
        d_dist = math.nan
        if self.distractor_pos is not None:
            d_dist = math.hypot(self.distractor_pos[0] - rx, self.distractor_pos[1] - ry)
            if self.estimate is not None:
                d_est = math.hypot(self.estimate[0] - self.distractor_pos[0],
                                   self.estimate[1] - self.distractor_pos[1])
                est_on_distractor = d_est < est_err
            else:
                est_on_distractor = False
        else:
            est_on_distractor = False

        amcl_err = math.nan
        if self.amcl is not None:
            mx, my, myaw = self.get_parameter("map_from_world").value
            ex = mx + math.cos(myaw) * rx - math.sin(myaw) * ry
            ey = my + math.sin(myaw) * rx + math.cos(myaw) * ry
            amcl_err = math.hypot(self.amcl[0] - ex, self.amcl[1] - ey)
        self.rows.append(dict(amcl_err=amcl_err, 
            t=round(t, 2), status=self.host_status, rx=rx, ry=ry, ryaw=ryaw,
            hx=hx, hy=hy, dist=dist, bearing=bearing, est_err=est_err,
            est_age=est_age, distractor_dist=d_dist, scan_min=self.scan_min,
            est_on_distractor=est_on_distractor,
        ))

        if self.host_status == "done" and self.done_at is None:
            self.done_at = now
        if (self.done_at is not None and now - self.done_at > self.settle_time) or (
            t > self.max_duration
        ):
            self._finish()

    # ---- output ----

    def _finish(self):
        self.finished = True
        os.makedirs(self.out_dir, exist_ok=True)
        with open(os.path.join(self.out_dir, "timeseries.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            w.writeheader()
            w.writerows(self.rows)
        summary = self.summarise()
        with open(os.path.join(self.out_dir, "summary.json"), "w") as f:
            json.dump(summary, f, indent=2)
        self.get_logger().info("SUMMARY " + json.dumps(summary))
        raise SystemExit

    def summarise(self):
        rows = self.rows
        dt = rows[1]["t"] - rows[0]["t"] if len(rows) > 1 else 0.1
        dist = np.array([r["dist"] for r in rows])
        walking = np.array([r["status"] == "walking" for r in rows])
        est_err = np.array([r["est_err"] for r in rows])
        est_age = np.array([r["est_age"] for r in rows])
        tracked = np.isfinite(est_err) & (est_err < 0.75) & (est_age < 1.0)
        on_distractor = np.array([r["est_on_distractor"] for r in rows])
        final = rows[-1]

        # Episodes where the robot fell behind beyond lost_distance.
        behind = dist > self.lost_distance
        behind_events = int(np.sum(behind[1:] & ~behind[:-1]) + (1 if behind[0] else 0))

        s = dict(
            duration=round(rows[-1]["t"], 1),
            dist_mean_walking=round(float(dist[walking].mean()), 2) if walking.any() else None,
            dist_max=round(float(dist.max()), 2),
            dist_min=round(float(dist.min()), 2),
            frac_dist_in_0_9_to_2_0_walking=round(
                float(((dist >= 0.9) & (dist <= 2.0))[walking].mean()), 3
            ) if walking.any() else None,
            frac_left_behind=round(float(behind.mean()), 3),
            left_behind_events=behind_events,
            final_dist=round(final["dist"], 2),
            arrived=bool(final["dist"] <= self.arrive_distance),
            min_scan_range=round(float(min(r["scan_min"] for r in rows)), 2),
            robot_travel=round(float(np.sum(np.hypot(
                np.diff([r["rx"] for r in rows]), np.diff([r["ry"] for r in rows])))), 2),
        )
        if np.isfinite(est_err).any():
            active = np.array([r["status"] != "done" for r in rows])  # scenario running
            s.update(
                frac_tracked=round(float(tracked[active].mean()), 3),
                est_err_median=round(float(np.nanmedian(est_err)), 2),
                frac_on_distractor=round(float(on_distractor.mean()), 3),
            )
        if not math.isnan(rows[0]["distractor_dist"]):
            s["distractor_min_dist"] = round(
                float(np.nanmin([r["distractor_dist"] for r in rows])), 2
            )
        amcl = np.array([r["amcl_err"] for r in rows])
        if np.isfinite(amcl).any():
            s["amcl_err_max"] = round(float(np.nanmax(amcl)), 2)
        s["sample_dt"] = round(dt, 3)
        return s


def main():
    rclpy.init()
    node = FollowEvaluator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        if not node.finished and node.rows:
            try:
                node._finish()
            except SystemExit:
                pass
    node.destroy_node()
    rclpy.try_shutdown()
