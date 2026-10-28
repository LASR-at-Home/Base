#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from control_msgs.action import PointHead, FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class PointHeadStub(Node):
    def __init__(self):
        super().__init__("point_head_stub")
        self.declare_parameter("pan_joint", "head_1_joint")
        self.declare_parameter("tilt_joint", "head_2_joint")
        self.declare_parameter("pan", 0.0)
        self.declare_parameter("tilt", -0.6)
        self.pan_joint = self.get_parameter("pan_joint").value
        self.tilt_joint = self.get_parameter("tilt_joint").value

        self._traj = ActionClient(
            self, FollowJointTrajectory, "/head_controller/follow_joint_trajectory")
        self._srv = ActionServer(
            self, PointHead, "/head_controller/point_head_action", self._on_goal)
        self.get_logger().info("point_head_stub ready → tilts head down on any goal")

    def _on_goal(self, goal_handle):
        pan = float(self.get_parameter("pan").value)
        tilt = float(self.get_parameter("tilt").value)
        self.get_logger().info(f"PointHead goal → head pan={pan}, tilt={tilt}")

        pt = JointTrajectoryPoint()
        pt.positions = [pan, tilt]
        pt.time_from_start = Duration(sec=1)
        traj = JointTrajectory()
        traj.joint_names = [self.pan_joint, self.tilt_joint]
        traj.points = [pt]

        fjt = FollowJointTrajectory.Goal()
        fjt.trajectory = traj
        if self._traj.wait_for_server(timeout_sec=3.0):
            self._traj.send_goal_async(fjt)
        else:
            self.get_logger().warn("head_controller/follow_joint_trajectory недоступний")

        goal_handle.succeed()
        return PointHead.Result()


def main():
    rclpy.init()
    node = PointHeadStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()