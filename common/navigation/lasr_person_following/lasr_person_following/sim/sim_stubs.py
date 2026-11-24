"""
Stand-ins for robot servers that the public TIAGo simulation does not provide
(sim testing only):

    /head_controller/point_head_action  control_msgs/PointHead
        Real implementation on top of /head_controller/follow_joint_trajectory: the
        target is transformed into torso_lift_link and converted to pan/tilt.
    transcribe_speech                   lasr_speech_recognition_interfaces/TranscribeSpeech
        Answers after `reply_delay` seconds, so voice confirmations can be exercised
        without a microphone. reply "oracle" (default) answers like the scripted host
        would to "have we arrived?": "yes" once /<host>/status is "done", else "no".
        Any other value is returned as is.
"""

import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import tf2_ros
from tf2_geometry_msgs import do_transform_point
from control_msgs.action import PointHead, FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from lasr_speech_recognition_interfaces.action import TranscribeSpeech

PAN_LIMITS = (-1.24, 1.24)
TILT_LIMITS = (-0.98, 0.72)


class SimStubs(Node):
    def __init__(self):
        super().__init__("sim_stubs")
        self.declare_parameter("reply", "oracle")
        self.declare_parameter("host", "test_person")
        self.host_status = None
        self.create_subscription(String, f"/{self.get_parameter('host').value}/status",
                                 lambda m: setattr(self, "host_status", m.data), 10)
        self.declare_parameter("reply_delay", 1.0)
        self.cb = ReentrantCallbackGroup()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.joints = {}
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        self.traj_client = ActionClient(
            self, FollowJointTrajectory, "/head_controller/follow_joint_trajectory",
            callback_group=self.cb,
        )
        self._latest_goal = 0
        self._lock = threading.Lock()
        ActionServer(
            self, PointHead, "/head_controller/point_head_action",
            execute_callback=self._point_head,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.cb,
        )
        ActionServer(
            self, TranscribeSpeech, "transcribe_speech",
            execute_callback=self._transcribe, callback_group=self.cb,
        )
        self.get_logger().info("Sim stubs ready (point_head_action, transcribe_speech)")

    def _js_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joints[name] = pos

    # ---- point head ----

    def _pan_tilt(self, target):
        t = self.tf_buffer.lookup_transform(
            "torso_lift_link", target.header.frame_id, rclpy.time.Time(),
            timeout=Duration(seconds=0.5),
        )
        p = do_transform_point(target, t).point
        h = self.tf_buffer.lookup_transform(
            "torso_lift_link", "head_2_link", rclpy.time.Time(),
            timeout=Duration(seconds=0.5),
        ).transform.translation
        dx, dy, dz = p.x - h.x, p.y - h.y, p.z - h.z
        pan = math.atan2(dy, dx)
        tilt = math.atan2(dz, math.hypot(dx, dy))  # positive = up on TIAGo
        return (
            min(max(pan, PAN_LIMITS[0]), PAN_LIMITS[1]),
            min(max(tilt, TILT_LIMITS[0]), TILT_LIMITS[1]),
        )

    def _point_head(self, goal_handle):
        with self._lock:
            self._latest_goal += 1
            my_id = self._latest_goal
        req = goal_handle.request
        try:
            pan, tilt = self._pan_tilt(req.target)
        except Exception as e:
            self.get_logger().warn(f"point_head: TF failed: {e}")
            goal_handle.abort()
            return PointHead.Result()

        cur_pan = self.joints.get("head_1_joint", 0.0)
        cur_tilt = self.joints.get("head_2_joint", 0.0)
        vel = req.max_velocity if req.max_velocity > 0 else 1.0
        min_dur = req.min_duration.sec + req.min_duration.nanosec * 1e-9
        dur = max(min_dur, max(abs(pan - cur_pan), abs(tilt - cur_tilt)) / vel, 0.15)

        if not self.traj_client.wait_for_server(timeout_sec=2.0):
            goal_handle.abort()
            return PointHead.Result()
        traj_goal = FollowJointTrajectory.Goal()
        traj_goal.trajectory.joint_names = ["head_1_joint", "head_2_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [pan, tilt]
        pt.time_from_start = Duration(seconds=dur).to_msg()
        traj_goal.trajectory.points = [pt]
        self.traj_client.send_goal_async(traj_goal)

        end = time.monotonic() + dur + 0.1
        while time.monotonic() < end:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return PointHead.Result()
            if self._latest_goal != my_id:  # pre-empted by a newer goal
                goal_handle.abort()
                return PointHead.Result()
            time.sleep(0.02)
        goal_handle.succeed()
        return PointHead.Result()

    # ---- speech ----

    def _transcribe(self, goal_handle):
        time.sleep(self.get_parameter("reply_delay").value)
        reply = self.get_parameter("reply").value
        if reply == "oracle":
            reply = "yes" if self.host_status == "done" else "no"
        self.get_logger().info(f"transcribe_speech stub replying: '{reply}'")
        goal_handle.succeed()
        return TranscribeSpeech.Result(sequence=reply)


def main():
    rclpy.init()
    node = SimStubs()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
