"""
look_down.py — YASMIN state: tilt TIAGo's head down (and pan) to a fixed angle,
so the depth camera looks into the basket before detection.

  LookDown()                      # default tilt -0.9 rad (look at floor ~0.8m ahead)
  LookDown(tilt=-0.75, pan=0.0)   # tune if the basket isn't centred in view

Sends a FollowJointTrajectory goal to the head controller and waits for it.
"""

import time

from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

try:
    from yasmin import State
except ImportError:
    class State:
        def __init__(self, outcomes):
            self._outcomes = outcomes


class LookDown(State):
    def __init__(self, pan=0.0, tilt=-1.0, secs=2, settle=1.0,
                 action='/head_controller/follow_joint_trajectory',
                 joints=('head_1_joint', 'head_2_joint')):
        super().__init__(outcomes=['succeeded', 'failed'])
        self.pan, self.tilt = pan, tilt
        self.secs, self.settle = secs, settle
        self.joints = joints
        from yasmin_ros.yasmin_node import YasminNode
        self.node = YasminNode.get_instance()
        self.cli = ActionClient(self.node, FollowJointTrajectory, action,
                                callback_group=ReentrantCallbackGroup())

    def _wait(self, future, timeout=15.0):
        t0 = time.time()
        while not future.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        return future.result() if future.done() else None

    def execute(self, blackboard):
        if not self.cli.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error('[LookDown] head action unavailable')
            return 'failed'
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(
            joint_names=list(self.joints),
            points=[JointTrajectoryPoint(
                positions=[float(self.pan), float(self.tilt)],
                time_from_start=Duration(sec=int(self.secs)))])
        gh = self._wait(self.cli.send_goal_async(goal))
        if gh is None or not gh.accepted:
            self.node.get_logger().error('[LookDown] goal rejected')
            return 'failed'
        self._wait(gh.get_result_async())
        if self.settle > 0:
            time.sleep(self.settle)   # let depth frames arrive at the new angle
        self.node.get_logger().info(f'[LookDown] head at pan={self.pan}, tilt={self.tilt}')
        return 'succeeded'