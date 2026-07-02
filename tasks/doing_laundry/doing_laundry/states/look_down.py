"""look_down.py — tilt TIAGo's head down via TOPIC (not action)."""

import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

try:
    from yasmin import State
except ImportError:
    class State:
        def __init__(self, outcomes):
            self._outcomes = outcomes


class LookDown(State):
    def __init__(self, pan=0.0, tilt=-0.9, secs=2, settle=2.0,
                 topic='/head_controller/joint_trajectory',
                 joints=('head_1_joint', 'head_2_joint')):
        super().__init__(outcomes=['succeeded', 'failed'])
        self.pan, self.tilt = pan, tilt
        self.secs, self.settle = secs, settle
        self.joints = joints
        from yasmin_ros.yasmin_node import YasminNode
        self.node = YasminNode.get_instance()
        self.pub = self.node.create_publisher(JointTrajectory, topic, 10)

    def execute(self, blackboard):
        msg = JointTrajectory(
            joint_names=list(self.joints),
            points=[JointTrajectoryPoint(
                positions=[float(self.pan), float(self.tilt)],
                time_from_start=Duration(sec=int(self.secs)))])
        for _ in range(5):
            self.pub.publish(msg)
            time.sleep(0.1)
        time.sleep(float(self.secs) + self.settle)
        self.node.get_logger().info(f'[LookDown] head tilt={self.tilt}')
        return 'succeeded'