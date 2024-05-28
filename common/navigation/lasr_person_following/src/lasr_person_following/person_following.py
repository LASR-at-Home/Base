import rospy

from leg_tracker.msg import PersonArray, Person

from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Pose,
    PoseStamped,
    Quaternion,
    PoseArray,
)

from typing import Union, List

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


import tf2_ros as tf
import tf2_geometry_msgs  # type: ignore
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose

from math import atan2
import numpy as np
from scipy.spatial.transform import Rotation as R


class PersonFollower:

    _track_sub: Union[None, rospy.Subscriber]
    _track_id: Union[None, int]
    _breadcrumbs: List[PoseStamped]
    _epsilon: float = 0.1
    _n_secs_static: float = 5.0
    _min_time_between_goals: float = 2.0
    _stopping_distance: float = 0.5
    _move_base_client: actionlib.SimpleActionClient
    _buffer: tf.Buffer
    _listener: tf.TransformListener
    _goal_pose_pub: rospy.Publisher

    def __init__(self):
        self._track_sub = None
        self._track_id = None
        self._breadcrumbs = []

        self._move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )

        self._buffer = tf.Buffer(cache_time=rospy.Duration(10.0))
        self._listener = tf.TransformListener(self._buffer)

        self._goal_pose_pub = rospy.Publisher(
            "/follow_poses", PoseArray, queue_size=1000000, latch=True
        )

    def _tf_pose(self, pose: PoseStamped, target_frame: str):
        trans = self._buffer.lookup_transform(
            target_frame, pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0)
        )
        return do_transform_pose(pose, trans)

    def _robot_pose_in_odom(self) -> PoseStamped:
        current_pose: PoseWithCovarianceStamped = rospy.wait_for_message(
            "/amcl_pose", PoseWithCovarianceStamped
        )
        current_pose_stamped = PoseStamped(
            pose=current_pose.pose.pose, header=current_pose.header
        )

        return self._tf_pose(current_pose_stamped, "odom")

    def begin_tracking(self) -> bool:
        """
        Chooses the closest person as the target
        """
        people: PersonArray = rospy.wait_for_message(
            "/people_tracked_throttled", PersonArray
        )

        if len(people.people) == 0:
            return False

        robot_pose = self._robot_pose_in_odom()

        closest_person: Person = min(
            people.people,
            key=lambda person: self._euclidian_distance(person.pose, robot_pose.pose),
        )

        self._track_id = closest_person.id

        rospy.loginfo(f"Tracking person with ID {self._track_id}")
        self._update_breadcrumbs(people)

        return True

    def _recover_track(self) -> bool:
        self._track_sub.unregister()

        people: PersonArray = rospy.wait_for_message(
            "/people_tracked_throttled", PersonArray
        )

        if len(people.people) == 0:
            return False

        if self._breadcrumbs:

            closest_person: Person = min(
                people.people,
                key=lambda person: self._euclidian_distance(
                    person.pose, self._breadcrumbs[0].pose
                ),
            )

        else:

            robot_pose: PoseStamped = self._robot_pose_in_odom()

            closest_person: Person = min(
                people.people,
                key=lambda person: self._euclidian_distance(
                    person.pose, robot_pose.pose
                ),
            )

        self._track_id = closest_person.id

        rospy.loginfo(f"Recovery: tracking person with ID {self._track_id}")
        self._breadcrumbs = []
        self._update_breadcrumbs(people)

        self._track_sub = rospy.Subscriber(
            "/people_tracked_throttled", PersonArray, self._update_breadcrumbs
        )

        return True

    def _update_breadcrumbs(self, tracks: PersonArray) -> None:
        relevant_tracks: List[Person] = [
            track for track in tracks.people if track.id == self._track_id
        ]
        poses = [
            (PoseStamped(pose=track.pose, header=tracks.header))
            for track in relevant_tracks
        ]

        self._breadcrumbs.extend(poses)

    def _euclidian_distance(self, p1: Pose, p2: Pose) -> float:
        return np.linalg.norm(
            np.array([p1.position.x, p1.position.y])
            - np.array([p2.position.x, p2.position.y])
        ).astype(float)

    def _compute_face_quat(self, p1: Pose, p2: Pose) -> Quaternion:
        dx: float = p2.position.x - p1.position.x
        dy: float = p2.position.y - p1.position.y
        theta_deg = np.degrees(atan2(dy, dx))
        x, y, z, w = R.from_euler("z", theta_deg, degrees=True).as_quat()
        return Quaternion(x, y, z, w)

    def _cancel_goal(self) -> None:
        if self._move_base_client.get_state() in [
            GoalStatus.PENDING,
            GoalStatus.ACTIVE,
        ]:
            self._move_base_client.cancel_goal()

    def follow(self) -> None:

        prev_track: Union[None, Person] = None
        prev_goal: Union[None, MoveBaseActionGoal] = None
        poses = []
        while not rospy.is_shutdown():
            tracks = rospy.wait_for_message("/people_tracked_throttled", PersonArray)
            current_track = next(
                filter(lambda track: track.id == self._track_id, tracks.people), None
            )
            if current_track is None:
                rospy.loginfo(f"Lost track of person with ID {self._track_id}")
                break

            if prev_track is not None:
                # If the poses are significantly different, update the most recent pose
                if self._euclidian_distance(prev_track.pose, current_track.pose) < 0.5:
                    rospy.loginfo("Person too close to previous one, skipping")
                    continue

            robot_pose = self._robot_pose_in_odom()

            if self._euclidian_distance(robot_pose.pose, current_track.pose) < 0.5:
                rospy.loginfo("Person too close to robot, skipping")
                continue

            goal_pose = PoseStamped(pose=current_track.pose, header=tracks.header)
            goal = MoveBaseGoal()
            goal.target_pose = self._tf_pose(goal_pose, "map")
            goal.target_pose.header.stamp = rospy.Time.now()

            poses.append(goal.target_pose.pose)
            self._move_base_client.send_goal(goal)

            prev_track = current_track

            pose_array = PoseArray()
            pose_array.header = goal.target_pose.header
            pose_array.poses = poses
            self._goal_pose_pub.publish(pose_array)
