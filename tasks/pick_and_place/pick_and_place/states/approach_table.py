import time
import math

import yasmin
import yasmin_ros

import rclpy
import rclpy.duration
from rclpy.action import ActionClient
from rclpy.time import Time as ROS2Time

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
from nav2_msgs.action import NavigateToPose

import tf2_ros


def _wait_future(future, timeout):
    """Wait on a future spun by the SM node's background executor."""
    deadline = time.time() + timeout
    while not future.done() and time.time() < deadline:
        time.sleep(0.02)
    return future.result() if future.done() else None


class ApproachTable(yasmin.State):
    """
    Drive to a standoff pose directly in FRONT of the detected table, facing it.

    Reads the table centre that AddTableCollision wrote to the blackboard
    (table_point, in map), takes the line from the robot to the table, and places
    a Nav2 goal `standoff` metres from the table centre along that line, oriented
    to look at the table. If navigation is rejected/fails (e.g. the goal is inside
    the table's costmap inflation), it backs off and retries at larger standoffs.

    Blackboard inputs:
        table_point : geometry_msgs/Point  - table centre in map

    Blackboard outputs:
        table_pose  : geometry_msgs/Pose   - the approach pose actually reached

    ROS 2 params (pick_and_place.approach):
        standoff          : float        - metres from table CENTRE (default 0.85).
                                           Smaller => closer (better arm reach) but
                                           Nav2 may refuse if too close to the table.
        retry_increments  : [float, ...] - extra standoff to add on retry
                                           (default [0.0, 0.15, 0.30, 0.45])
        nav_timeout       : float        - per-attempt nav timeout (default 120)

    Outcomes: succeeded, failed
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("table_point")
        self.add_output_key("table_pose")

        self.node = yasmin_ros.logger_node
        self._tf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf, self.node)
        self._nav = ActionClient(self.node, NavigateToPose, "navigate_to_pose")

    def _param(self, name, default):
        try:
            v = self.node.get_parameter(name).value
            return v if v is not None else default
        except Exception:
            return default

    def _robot_xy(self, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                tf = self._tf.lookup_transform(
                    "map", "base_footprint", ROS2Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
                return tf.transform.translation.x, tf.transform.translation.y
            except Exception:
                time.sleep(0.2)
        return None

    def _navigate(self, pose, timeout):
        if not self._nav.wait_for_server(timeout_sec=5.0):
            yasmin.YASMIN_LOG_ERROR("Nav2 navigate_to_pose server not available.")
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped(header=Header(frame_id="map"), pose=pose)
        gh = _wait_future(self._nav.send_goal_async(goal), 5.0)
        if gh is None or not gh.accepted:
            yasmin.YASMIN_LOG_WARN("Nav goal rejected.")
            return False
        res = _wait_future(gh.get_result_async(), timeout)
        if res is None:
            yasmin.YASMIN_LOG_WARN("Navigation timed out.")
            return False
        if res.status != 4:                 # 4 = SUCCEEDED (action_msgs/GoalStatus)
            yasmin.YASMIN_LOG_WARN(f"Navigation failed (status {res.status}).")
            return False
        return True

    def execute(self, blackboard) -> str:
        try:
            tp = blackboard["table_point"]
        except Exception:
            tp = None
        if tp is None:
            yasmin.YASMIN_LOG_WARN("No table_point in blackboard — cannot approach.")
            return "failed"

        standoff = float(self._param("pick_and_place.approach.standoff", 0.85))
        incs = list(self._param(
            "pick_and_place.approach.retry_increments", [0.0, 0.15, 0.30, 0.45]
        ))
        nav_timeout = float(self._param("pick_and_place.approach.nav_timeout", 120.0))

        rxy = self._robot_xy()
        if rxy is None:
            yasmin.YASMIN_LOG_ERROR("No robot pose (TF map->base_footprint).")
            return "failed"
        rx, ry = rxy

        dx, dy = rx - tp.x, ry - tp.y
        dist = math.hypot(dx, dy)
        ux, uy = (1.0, 0.0) if dist < 1e-3 else (dx / dist, dy / dist)
        yasmin.YASMIN_LOG_INFO(
            f"Table at map=({tp.x:.2f},{tp.y:.2f}); robot=({rx:.2f},{ry:.2f}); "
            f"current gap={dist:.2f} m."
        )

        for inc in incs:
            s = standoff + float(inc)
            gx, gy = tp.x + ux * s, tp.y + uy * s
            yaw = math.atan2(tp.y - gy, tp.x - gx)   # face the table
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = gx, gy, 0.0
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            yasmin.YASMIN_LOG_INFO(
                f"Approach: standoff={s:.2f} goal=({gx:.2f},{gy:.2f}) yaw={yaw:.2f}"
            )
            if self._navigate(pose, nav_timeout):
                blackboard["table_pose"] = pose
                yasmin.YASMIN_LOG_INFO("Reached table approach pose.")
                return "succeeded"

        yasmin.YASMIN_LOG_WARN("All approach standoffs failed.")
        return "failed"