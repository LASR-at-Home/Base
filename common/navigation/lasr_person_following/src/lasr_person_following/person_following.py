import rospy

from leg_tracker.msg import PersonArray, Person

from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Pose,
    PoseStamped,
    Quaternion,
    PoseArray,
    Point,
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
    _n_secs_static_finished: float
    _n_secs_static_plan_close: float
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

    # Publishers
    _person_trajectory_pub: rospy.Publisher

    def __init__(
        self,
        start_following_radius: float = 2.0,
        start_following_angle: float = 45.0,
        min_distance_between_tracks: float = 0.5,
        n_secs_static_finished: float = 10.0,
        n_secs_static_plan_close: float = 5.0,
        new_goal_threshold: float = 1.0,
        stopping_distance: float = 1.0,
    ):
        self._start_following_radius = start_following_radius
        self._start_following_angle = start_following_angle
        self._min_distance_between_tracks = min_distance_between_tracks
        self._n_secs_static_finished = n_secs_static_finished
        self._n_secs_static_plan_close = n_secs_static_plan_close
        self._new_goal_threshold = new_goal_threshold
        self._stopping_distance = stopping_distance

        self._track_id = None

        self._move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        if not self._move_base_client.wait_for_server(rospy.Duration.from_sec(10.0)):
            rospy.logwarn("Move base client not available")

        self._buffer = tf.Buffer(cache_time=rospy.Duration.from_sec(10.0))
        self._listener = tf.TransformListener(self._buffer)

        rospy.wait_for_service(
            "/move_base/make_plan", timeout=rospy.Duration.from_sec(10.0)
        )
        self._make_plan = rospy.ServiceProxy("/move_base/make_plan", GetPlan)

        self._person_trajectory_pub = rospy.Publisher(
            "/person_trajectory", PoseArray, queue_size=1, latch=True
        )

        self._tts_client = actionlib.SimpleActionClient("tts", TtsAction)
        self._tts_client_available = self._tts_client.wait_for_server(
            rospy.Duration.from_sec(10.0)
        )
        if not self._tts_client_available:
            rospy.logwarn("TTS client not available")
        self._transcribe_speech_client = actionlib.SimpleActionClient(
            "transcribe_speech", TranscribeSpeechAction
        )
        self._transcribe_speech_client_available = (
            self._transcribe_speech_client.wait_for_server(
                rospy.Duration.from_sec(10.0)
            )
        )
        if not self._transcribe_speech_client_available:
            rospy.logwarn("Transcribe speech client not available")

    def _tf_pose(self, pose: PoseStamped, target_frame: str):
        trans = self._buffer.lookup_transform(
            target_frame, pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0)
        )
        return do_transform_pose(pose, trans)

    def _robot_pose_in_odom(self) -> Union[PoseStamped, None]:
        try:
            current_pose: PoseWithCovarianceStamped = rospy.wait_for_message(
                "/amcl_pose", PoseWithCovarianceStamped
            )
        except AttributeError:
            return None

        current_pose_stamped = PoseStamped(
            pose=current_pose.pose.pose, header=current_pose.header
        )

        return self._tf_pose(current_pose_stamped, "odom")

    def begin_tracking(self, ask: bool = False) -> bool:
        """
        Chooses the closest person as the target
        """

        tracks: PersonArray = rospy.wait_for_message("/people_tracked", PersonArray)
        people: List[Person] = tracks.people

        if len(people) == 0:
            return False

        min_dist: float = np.inf
        closest_person: Union[None, Person] = None
        robot_pose: PoseStamped = self._robot_pose_in_odom()

        if robot_pose is None:
            return False

        for person in people:
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

        if ask:
            if self._tts_client_available and self._transcribe_speech_client_available:

                # Ask if we should follow
                self._tts("Should I follow you? Please answer yes or no", wait=True)

                # listen
                self._transcribe_speech_client.send_goal_and_wait(
                    TranscribeSpeechGoal()
                )
                transcription = self._transcribe_speech_client.get_result().sequence

                if "yes" not in transcription.lower():
                    return False

                self._tts("I will follow you", wait=False)

        self._track_id = closest_person.id

        rospy.loginfo(f"Tracking person with ID {self._track_id}")
        return True

    def _recover_track(self) -> bool:
        if self._tts_client_available:
            self._tts("I lost track of you, please come back", wait=True)

        while not self.begin_tracking(ask=True) and not rospy.is_shutdown():
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
            self._move_base_client.wait_for_result()

    def _check_finished(self) -> bool:
        if self._tts_client_available:
            self._tts("Have we arrived?", wait=True)

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
        start: rospy.Time = rospy.Time.now()
        assert (
            start_pose.header.frame_id == goal_pose.header.frame_id
        ), "Start and goal poses must be in the same frame"

        chosen_pose: Union[None, PoseStamped] = None

        # make a plan from start to goal
        try:
            plan = self._make_plan(start_pose, goal_pose, self._stopping_distance).plan
        except rospy.ServiceException as e:
            rospy.loginfo(e)
            return chosen_pose

        # select a pose that is dist_to_goal away from the goal_pose
        for pose in reversed(plan.poses):
            if self._euclidian_distance(pose.pose, goal_pose.pose) >= dist_to_goal:
                chosen_pose = pose
                break
        end: rospy.Time = rospy.Time.now()

        rospy.logwarn(f"Time taken to find a plan: {end.secs - start.secs} seconds")

        return chosen_pose

    def _move_base(self, pose: PoseStamped) -> MoveBaseGoal:
        goal: MoveBaseGoal = MoveBaseGoal()
        goal.target_pose = pose
        self._move_base_client.send_goal(goal)

        return goal

    def _tts(self, text: str, wait: bool) -> None:
        if self._tts_client_available:
            tts_goal: TtsGoal = TtsGoal()
            tts_goal.rawtext.text = text
            tts_goal.rawtext.lang_id = "en_GB"
            if wait:
                self._tts_client.send_goal_and_wait(tts_goal)
            else:
                self._tts_client.send_goal(tts_goal)

    def follow(self) -> None:

        person_trajectory: PoseArray = PoseArray()
        person_trajectory.header.frame_id = "odom"
        prev_track: Union[None, Person] = None
        last_goal_time: Union[None, rospy.Time] = None
        going_to_person: bool = False

        while not rospy.is_shutdown():

            tracks: PersonArray = rospy.wait_for_message("/people_tracked", PersonArray)

            # Get the most recent track of the person we are following
            track = next(
                filter(lambda track: track.id == self._track_id, tracks.people),
                None,
            )

            if track is None:
                rospy.loginfo("Lost track of person, recovering...")
                self._cancel_goal()
                person_trajectory = PoseArray()
                self._recover_track()
                prev_track = None
                continue

            # Distance to the previous pose
            dist_to_prev = (
                self._euclidian_distance(track.pose, prev_track.pose)
                if prev_track is not None
                else np.inf
            )

            # Check if the person has moved significantly
            if dist_to_prev >= self._new_goal_threshold:

                goal_pose = self._tf_pose(
                    PoseStamped(pose=track.pose, header=tracks.header),
                    "map",
                )
                self._move_base(goal_pose)
                prev_track = track
                last_goal_time = rospy.Time.now()
                person_trajectory.poses.append(track.pose)
                person_trajectory.header.stamp = rospy.Time.now()
                self._person_trajectory_pub.publish(person_trajectory)
                going_to_person = False
            elif last_goal_time is not None:
                delta_t: float = (rospy.Time.now() - last_goal_time).to_sec()
                if (
                    self._n_secs_static_plan_close
                    <= delta_t
                    < self._n_secs_static_finished
                ) and not going_to_person:
                    self._cancel_goal()
                    goal_pose = self._get_pose_on_path(
                        self._tf_pose(self._robot_pose_in_odom(), "map"),
                        self._tf_pose(
                            PoseStamped(pose=track.pose, header=tracks.header),
                            "map",
                        ),
                        self._stopping_distance,
                    )
                    if goal_pose is not None:
                        self._move_base(goal_pose)
                        going_to_person = True
                    else:
                        rospy.logwarn("Could not find a path to the person")
                elif delta_t >= self._n_secs_static_finished:
                    rospy.loginfo(
                        "Person has been static for too long, checking if we are finished"
                    )
                    self._move_base_client.wait_for_result()
                    if self._check_finished():
                        rospy.loginfo("Finished following person")
                        break
                    else:
                        rospy.loginfo("Person is not finished, continuing")
                        last_goal_time = None
                        continue
