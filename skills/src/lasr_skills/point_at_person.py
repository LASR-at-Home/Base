from typing import Optional

import rospy
import smach

from lasr_skills import PlayMotion

from geometry_msgs.msg import (
    Pose,
    Point,
    PoseWithCovarianceStamped,
    Quaternion,
    PoseStamped,
)
from tf.transformations import quaternion_from_matrix
import numpy as np

import moveit_commander

import tf2_geometry_msgs
import tf2_ros


class PointAt(smach.State):
    def __init__(
        self,
        target_point: Optional[Point] = None,
        eef_height: float = 0.6,
        max_dist_from_base: float = 0.75,
    ):
        smach.State.__init__(self, outcomes=["succeeded", "aborted"])

        self._target = target_point
        self._eef_height = eef_height
        self._max_dist = max_dist_from_base

        self._group = moveit_commander.MoveGroupCommander("arm_torso")
        self._group.set_planner_id("RRTConnectkConfigDefault")
        self._group.allow_replanning(True)
        self._group.allow_looking(True)
        self._group.set_planning_time(30)
        self._group.set_num_planning_attempts(30)
        self._group.set_pose_reference_frame("base_footprint")

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    def compute_eef_pose(self, robot_pose: Pose, target_point: Point) -> PoseStamped:
        robot_pos = np.array(
            [
                robot_pose.position.x,
                robot_pose.position.y,
                robot_pose.position.z,
            ]
        )
        rospy.logdebug(f"Robot pose: {robot_pose}, Target point: {target_point}")
        target_pos = np.array(
            [
                target_point.x,
                target_point.y,
                target_point.z,
            ]
        )

        # Direction vector
        direction = target_pos - robot_pos
        distance = np.linalg.norm(direction)
        if distance == 0:
            rospy.logwarn(
                "Target point coincides with robot position. Returning default pose."
            )
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose = Pose(
                position=Point(
                    robot_pose.position.x, robot_pose.position.y, self._eef_height
                ),
                orientation=Quaternion(0, 0, 0, 1),
            )
            return pose_stamped
        direction /= distance

        position = robot_pos + direction * min(distance, self._max_dist)
        position[2] = self._eef_height

        x_axis = direction
        up = np.array([0, 0, 1])
        if abs(np.dot(x_axis, up)) > 0.99:
            up = np.array([0, 1, 0])

        z_axis = np.cross(x_axis, up)
        z_axis /= np.linalg.norm(z_axis)
        y_axis = np.cross(z_axis, x_axis)

        rot = np.eye(4)
        rot[0:3, 0] = x_axis
        rot[0:3, 1] = y_axis
        rot[0:3, 2] = z_axis

        quat = quaternion_from_matrix(rot)

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = Pose(
            position=Point(*position), orientation=Quaternion(*quat)
        )

        return pose_stamped

    def execute(self, ud):
        target = self._target or ud.target
        robot_pose = rospy.wait_for_message(
            "/amcl_pose", PoseWithCovarianceStamped
        ).pose.pose
        eef_pose = self.compute_eef_pose(robot_pose, target)

        eef_pose = self._tf_buffer.transform(
            eef_pose, "base_footprint", rospy.Duration(5.0)
        )

        self._group.set_pose_target(
            eef_pose, end_effector_link="gripper_grasping_frame"
        )
        success = self._group.go(wait=True)
        self._group.stop()
        self._group.clear_pose_targets()

        return "succeeded" if success else "aborted"


class PointAtPerson(smach.StateMachine):
    def __init__(
        self,
        target_point: Optional[Point] = None,
        eef_height: float = 0.6,
        max_dist_from_base: float = 0.75,
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["target"],
        )

        with self:
            smach.StateMachine.add(
                "PRE_POINT",
                PlayMotion("pre_point"),
                transitions={
                    "succeeded": "POINT",
                    "preempted": "POINT",
                    "aborted": "POINT",
                },
            )
            smach.StateMachine.add(
                "POINT",
                PointAt(target_point, eef_height, max_dist_from_base),
                transitions={"succeeded": "succeeded", "aborted": "failed"},
            )
