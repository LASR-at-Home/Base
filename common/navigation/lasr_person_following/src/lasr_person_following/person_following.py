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

import rosservice
import tf2_ros as tf
import tf2_geometry_msgs  # type: ignore
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path


from math import atan2
import numpy as np
from scipy.spatial.transform import Rotation as R

import actionlib
from pal_interaction_msgs.msg import TtsGoal, TtsAction
from lasr_speech_recognition_msgs.msg import (
    TranscribeSpeechAction,
    TranscribeSpeechGoal,
)


class PersonFollower:

    # Parameters
    _start_following_radius: float
    _start_following_angle: float
    _min_distance_between_tracks: float
    _n_secs_static: float
    _min_time_between_goals: float
    _new_goal_threshold: float
    _stopping_distance: float

    # State
    _latest_tracks: Union[None, PersonArray]
    _track_id: Union[None, int]

    # Action clients
    _move_base_client: actionlib.SimpleActionClient
    _tts_client: actionlib.SimpleActionClient
    _tts_client_available: bool
    _transcribe_speech_client: actionlib.SimpleActionClient
    _transcribe_speech_client_available: bool

    # Services
    _make_plan: rospy.ServiceProxy

    # Tf
    _buffer: tf.Buffer
    _listener: tf.TransformListener

    # Subscribers
    _track_sub: rospy.Subscriber

    # Publishers
    _goal_pose_pub: rospy.Publisher

    def __init__(
        self,
        start_following_radius: float = 2.0,
        start_following_angle: float = 45.0,
        min_distance_between_tracks: float = 0.5,
        n_secs_static: float = 15.0,
        min_time_between_goals: float = 1.0,
        new_goal_threshold: float = 0.5,
        stopping_distance: float = 1.0,
    ):
        self._start_following_radius = start_following_radius
        self._start_following_angle = start_following_angle
        self._min_distance_between_tracks = min_distance_between_tracks
        self._n_secs_static = n_secs_static
        self._min_time_between_goals = min_time_between_goals
        self._new_goal_threshold = new_goal_threshold
        self._stopping_distance = stopping_distance

        self._latest_tracks = None
        self._track_id = None

        self._move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )

        self._buffer = tf.Buffer(cache_time=rospy.Duration(10.0))
        self._listener = tf.TransformListener(self._buffer)

        self._goal_pose_pub = rospy.Publisher(
            "/follow_poses", PoseArray, queue_size=1000000, latch=True
        )

        self._track_sub = rospy.Subscriber(
            "/people_tracked", PersonArray, self._track_callback
        )

        """
        Apparently, /move_base/NavfnROS/make_plan can be called while move_base is active https://github.com/ros-planning/navigation/issues/12
        """
        rospy.wait_for_service("/move_base/make_plan")
        self._make_plan = rospy.ServiceProxy("/move_base/make_plan", GetPlan)

        self._tts_client = actionlib.SimpleActionClient("/tts", TtsAction)
        self._tts_client_available = self._tts_client.wait_for_server(
            rospy.Duration(10.0)
        )
        self._transcribe_speech_client = actionlib.SimpleActionClient(
            "transcribe_speech", TranscribeSpeechAction
        )
        self._transcribe_speech_client_available = (
            self._transcribe_speech_client.wait_for_server(rospy.Duration(10.0))
        )

    def _track_callback(self, msg: PersonArray) -> None:
        self._latest_tracks = msg

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
        while self._latest_tracks is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for people to be tracked")
            rospy.sleep(1)

        if self._latest_tracks is None:
            rospy.loginfo("No people to track")
            return False

        people: PersonArray = self._latest_tracks

        if len(people.people) == 0:
            return False

        min_dist: float = np.inf
        closest_person: Union[None, Person] = None
        robot_pose: PoseStamped = self._robot_pose_in_odom()
        for person in people.people:
            dist: float = self._euclidian_distance(person.pose, robot_pose.pose)

            face_quat: Quaternion = self._compute_face_quat(
                robot_pose.pose, person.pose
            )
            robot_dir: np.ndarray = self._quat_to_dir(robot_pose.pose.orientation)
            person_dir: np.ndarray = self._quat_to_dir(face_quat)
            angle: float = np.degrees(
                self._angle_between_vectors(robot_dir, person_dir)
            )
            rospy.loginfo(f"Person {person.id} is at {dist}m and {angle} degrees")
            if (
                dist < self._start_following_radius
                and abs(angle) < self._start_following_angle
            ):
                if dist < min_dist:
                    min_dist = dist
                    closest_person = person

        if not closest_person:
            return False

        # Face the person
        pose: PoseStamped = PoseStamped(header=people.header)
        pose.pose = robot_pose.pose
        pose.pose.orientation = self._compute_face_quat(
            robot_pose.pose, closest_person.pose
        )
        goal: MoveBaseGoal = MoveBaseGoal()
        goal.target_pose = self._tf_pose(pose, "map")
        goal.target_pose.header.stamp = rospy.Time.now()
        self._move_base_client.send_goal(goal)

        if self._tts_client_available:
            # Ask if we should follow
            tts_goal: TtsGoal = TtsGoal()
            tts_goal.rawtext.text = "Should I follow you? Please answer yes or no"
            tts_goal.rawtext.lang_id = "en_GB"
            self._tts_client.send_goal_and_wait(tts_goal)

            if self._transcribe_speech_client_available:
                # listen
                self._transcribe_speech_client.send_goal_and_wait(
                    TranscribeSpeechGoal()
                )
                transcription = self._transcribe_speech_client.get_result().sequence

                if "yes" not in transcription.lower():
                    return False

            tts_goal: TtsGoal = TtsGoal()
            tts_goal.rawtext.text = (
                "Okay, I'll begin following you. Please walk slowly."
            )
            tts_goal.rawtext.lang_id = "en_GB"
            self._tts_client.send_goal_and_wait(tts_goal)

        self._track_id = closest_person.id

        rospy.loginfo(f"Tracking person with ID {self._track_id}")
        return True

    def _recover_track(self) -> bool:
        if self._tts_client_available:
            tts_goal: TtsGoal = TtsGoal()
            tts_goal.rawtext.text = "I've lost you, please come back"
            tts_goal.rawtext.lang_id = "en_GB"
            self._tts_client.send_goal_and_wait(tts_goal)

        while not self.begin_tracking() and not rospy.is_shutdown():
            rospy.loginfo("Recovering track...")
            rospy.sleep(1)

        return True

    def _euclidian_distance(self, p1: Pose, p2: Pose) -> float:
        return np.linalg.norm(
            np.array([p1.position.x, p1.position.y])
            - np.array([p2.position.x, p2.position.y])
        ).astype(float)

    def _quat_to_dir(self, q: Quaternion):
        x, y, z, w = q.x, q.y, q.z, q.w
        forward = np.array(
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)]
        )
        return forward

    def _angle_between_vectors(self, v1, v2):
        dot_product = np.dot(v1, v2)
        norms_product = np.linalg.norm(v1) * np.linalg.norm(v2)
        cos_theta = dot_product / norms_product
        cos_theta = np.clip(cos_theta, -1.0, 1.0)
        angle = np.arccos(cos_theta)
        return angle

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

    def _check_finished(self) -> bool:
        if self._tts_client_available:
            tts_goal: TtsGoal = TtsGoal()
            tts_goal.rawtext.text = (
                "Should I stop following you? Please answer yes or no"
            )
            tts_goal.rawtext.lang_id = "en_GB"
            self._tts_client.send_goal_and_wait(tts_goal)

            if self._transcribe_speech_client_available:
                self._transcribe_speech_client.send_goal_and_wait(
                    TranscribeSpeechGoal()
                )
                transcription = self._transcribe_speech_client.get_result().sequence

                if "yes" in transcription.lower():
                    return True

            return False
        return True

    def _get_pose_on_path(
        self, start_pose: PoseStamped, goal_pose: PoseStamped, dist_to_goal: float
    ) -> Union[None, PoseStamped]:
        assert (
            start_pose.header.frame_id == goal_pose.header.frame_id
        ), "Start and goal poses must be in the same frame"

        chosen_pose: Union[None, PoseStamped] = None

        # make a plan from start to goal
        try:
            plan = self._make_plan(start_pose, goal_pose, self._stopping_distance).plan
        except rospy.ServiceException:
            return chosen_pose

        # select a pose that is dist_to_goal away from the goal_pose
        for pose in reversed(plan.poses):
            if self._euclidian_distance(pose.pose, goal_pose.pose) >= dist_to_goal:
                chosen_pose = pose
                break

        return chosen_pose

    def follow(self) -> None:
        """
        The general loop should be:
            - Get persons pose
            - Go to pose - stopping_distance
                - If person moves significantly, cancel goal and go to new pose
            - If person's pose is too close to the previous pose, skip
            - If person is too close to the robot, face them and skip
            - If we are currently executing a goal, it is too soon to send a new one, and the person has not moved significantly, skip
            - If the person has been static for n_secs_static seconds, check if they are finished

        """

        prev_track: Union[None, Person] = None
        last_track_time: Union[None, rospy.Time] = None
        prev_goal: Union[None, MoveBaseActionGoal] = None

        while not rospy.is_shutdown():

            # No tracks, so skip
            if self._latest_tracks is None:
                rospy.loginfo("No tracks, skipping")
                continue
            # Get the most recent track of the person we are following
            track = next(
                filter(
                    lambda track: track.id == self._track_id, self._latest_tracks.people
                ),
                None,
            )

            # If we have lost track of the person, finish executing the curret goal and recover the track
            if track is None:
                self._move_base_client.wait_for_result()
                rospy.loginfo("Lost track of person, recovering...")
                self._recover_track()
                continue

            # Get the robot's pose in the odom frame
            robot_pose = self._robot_pose_in_odom()

            # Distance to the previous pose
            dist_to_prev = (
                self._euclidian_distance(track.pose, prev_track.pose)
                if prev_track is not None
                else np.inf
            )

            # Distance to the goal
            dist_to_goal = self._euclidian_distance(robot_pose.pose, track.pose)

            # Check if the person is too close to the previous pose
            if dist_to_prev < self._min_distance_between_tracks:
                rospy.loginfo("Person too close to previous, skipping")
                # prev_track = track
                if last_track_time is not None:
                    if (
                        rospy.Time.now().secs - last_track_time.secs
                        > self._n_secs_static
                    ):
                        if self._check_finished():
                            break
                        else:
                            last_track_time = None
                            continue
                continue

            # Check if the person is too close to the robot

            if dist_to_goal < self._stopping_distance:
                rospy.loginfo("Person too close to robot, facing them and skipping")
                pose = PoseStamped(header=self._latest_tracks.header)
                pose.pose = robot_pose.pose
                pose.pose.orientation = self._compute_face_quat(
                    robot_pose.pose, track.pose
                )
                goal = MoveBaseGoal()
                goal.target_pose = self._tf_pose(pose, "map")
                goal.target_pose.header.stamp = rospy.Time.now()
                self._move_base_client.send_goal(goal)
                # prev_track = track
                continue

            time_since_last_goal: float = (
                rospy.Time.now().secs - prev_goal.target_pose.header.stamp.secs
                if prev_goal is not None
                else np.inf
            )

            # Check if the person has moved significantly
            if dist_to_prev > self._new_goal_threshold:
                rospy.loginfo("Person has moved significantly, cancelling goal")
                self._cancel_goal()
            # Check if we are currently executing a goal, it is too soon to send a new one, and the person has not moved significantly
            elif (
                self._move_base_client.get_state()
                in [
                    GoalStatus.PENDING,
                    GoalStatus.ACTIVE,
                ]
                and time_since_last_goal < self._min_time_between_goals
            ):
                rospy.loginfo("Currently executing a goal, skipping")
                prev_track = track
                last_track_time = rospy.Time.now()
                continue

            goal: MoveBaseGoal = MoveBaseGoal()
            goal.target_pose = self._get_pose_on_path(
                self._tf_pose(robot_pose, "map"),
                self._tf_pose(
                    PoseStamped(pose=track.pose, header=self._latest_tracks.header),
                    "map",
                ),
                self._stopping_distance,
            )
            if goal.target_pose is None:
                rospy.loginfo("Failed to find a plan, skipping")
                continue
            goal.target_pose.header.stamp = rospy.Time.now()
            self._move_base_client.send_goal(goal)
            rospy.loginfo(f"Sent goal to {goal.target_pose.pose.position}")
            prev_goal = goal
            prev_track = track
            last_track_time = rospy.Time.now()
