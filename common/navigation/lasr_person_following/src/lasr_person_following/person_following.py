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

from typing import Union, List, Tuple

from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import PointCloud2

import rosservice
import tf2_ros as tf
import tf2_geometry_msgs  # type: ignore
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from nav_msgs.srv import GetPlan
from lasr_vision_msgs.srv import DetectWave
from play_motion_msgs.msg import PlayMotionAction
from play_motion_msgs.msg import PlayMotionGoal

from lasr_vision_msgs.srv import DetectWaveRequest, DetectWaveResponse

from math import atan2
import numpy as np
from scipy.spatial.transform import Rotation as R

import actionlib
from lasr_speech_recognition_msgs.msg import (
    TranscribeSpeechAction,
    TranscribeSpeechGoal,
)

from pal_interaction_msgs.msg import TtsGoal, TtsAction

from lasr_person_following.msg import (
    FollowAction,
    FollowGoal,
    FollowResult,
    FollowFeedback,
)

MAX_VISION_DIST: float = 5.0


class PersonFollower:
    # Parameters
    _start_following_radius: float
    _start_following_angle: float
    _n_secs_static_finished: float
    _n_secs_static_plan_close: float
    _new_goal_threshold: float
    _stopping_distance: float
    _vision_recovery_motions: List[str] = ["look_centre", "look_left", "look_right"]
    _vision_recovery_attempts: int = 3

    # State
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

    # Waypoints
    _waypoints: List[PoseStamped]

    def __init__(
        self,
        start_following_radius: float = 2.0,
        start_following_angle: float = 45.0,
        new_goal_threshold: float = 2.0,
        stopping_distance: float = 1.0,
        static_speed: float = 0.0015,
        max_speed: float = 0.55,
        asking_time_limit: float = 15.0,
    ):
        self._start_following_radius = start_following_radius
        self._start_following_angle = start_following_angle
        self._new_goal_threshold = new_goal_threshold
        self._stopping_distance = stopping_distance
        self._static_speed = static_speed
        self._max_speed = max_speed
        self._asking_time_limit = asking_time_limit

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

        self._detect_wave = rospy.ServiceProxy("/bodypix/detect_wave", DetectWave)
        if not self._detect_wave.wait_for_service(rospy.Duration.from_sec(10.0)):
            rospy.logwarn("Detect wave service not available")

        self._play_motion = actionlib.SimpleActionClient(
            "play_motion", PlayMotionAction
        )
        if not self._play_motion.wait_for_server(rospy.Duration.from_sec(10.0)):
            rospy.logwarn("Play motion client not available")

        self._waypoints = []

    def _tf_pose(self, pose: PoseStamped, target_frame: str):
        trans = self._buffer.lookup_transform(
            target_frame, pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0)
        )
        return do_transform_pose(pose, trans)

    def _robot_pose_in_odom(self) -> Union[PoseStamped, None]:
        try:
            current_pose: PoseWithCovarianceStamped = rospy.wait_for_message(
                "/robot_pose",
                PoseWithCovarianceStamped,
                timeout=rospy.Duration.from_sec(2.0),
            )
        except rospy.ROSException:
            return None
        except AttributeError:
            return None

        current_pose_stamped = PoseStamped(
            pose=current_pose.pose.pose, header=current_pose.header
        )

        return self._tf_pose(current_pose_stamped, "odom")

    def begin_tracking(self) -> bool:
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

        self._track_id = closest_person.id

        rospy.loginfo(f"Tracking person with ID {self._track_id}")
        return True

    def _recover_track(self, say) -> bool:
        if not say:
            self._tts("I see you are waving", wait=True)

        if self._tts_client_available and say:
            self._tts("Please could you come back...", wait=True)

        while not self.begin_tracking() and not rospy.is_shutdown():
            rospy.loginfo("Recovering track...")
            rospy.sleep(1)

        self._tts("I see you again", wait=False)

        return True

    def _detect_waving_person(self) -> DetectWaveResponse:
        try:
            pcl = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
            req = DetectWaveRequest()
            req.pcl_msg = pcl
            req.dataset = "resnet50"
            req.confidence = 0.1
            response = self._detect_wave(req)
            return response
        except rospy.ServiceException as e:
            rospy.loginfo(f"Error detecting wave: {e}")
            return DetectWaveResponse()

    # recover with vision, look around and check if person is waving
    def _recover_vision(self, robot_pose: PoseStamped, prev_pose: PoseStamped) -> bool:

        # cancel current goal
        self._cancel_goal()

        self._tts("Please come into my view and wave so that I can find you", wait=True)

        for motion in self._vision_recovery_motions:
            rospy.loginfo(f"Performing motion: {motion}")
            goal = PlayMotionGoal()
            goal.motion_name = motion
            self._play_motion.send_goal_and_wait(goal)
            for _ in range(self._vision_recovery_attempts):
                response = self._detect_waving_person()
                if response.wave_detected:
                    if np.isnan(response.wave_position.point.x) or np.isnan(
                        response.wave_position.point.y
                    ):
                        continue
                    else:
                        goal_pose = self._tf_pose(
                                PoseStamped(
                                    pose=Pose(
                                        position=Point(
                                            x=response.wave_position.point.x,
                                            y=response.wave_position.point.y,
                                            z=response.wave_position.point.z,
                                        ),
                                        orientation=Quaternion(0, 0, 0, 1),
                                    ),
                                    header=prev_pose.header,
                                ), "map"
                        )
                        if goal_pose is None:
                            rospy.logwarn("Could not find a path to the person")
                            return False
                        else:
                            self._move_base(goal_pose)
                            return True
        return False

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

                return "yes" in transcription.lower()
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

    def _move_base(self, pose: PoseStamped, wait=False) -> MoveBaseGoal:
        goal: MoveBaseGoal = MoveBaseGoal()
        goal.target_pose = pose
        self._move_base_client.send_goal(goal)

        if wait:
            self._move_base_client.wait_for_result()

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

    def follow(self) -> FollowResult:

        result = FollowResult()

        person_trajectory: PoseArray = PoseArray()
        person_trajectory.header.frame_id = "odom"
        prev_track: Union[None, Person] = None
        prev_goal: Union[None, PoseStamped] = None
        last_goal_time: Union[None, rospy.Time] = None
        going_to_person: bool = False
        track_vels: List[Tuple[float, float]] = []
        asking_time: rospy.Time = rospy.Time.now()

        while not rospy.is_shutdown():

            tracks: PersonArray = rospy.wait_for_message("/people_tracked", PersonArray)

            # Get the most recent track of the person we are following
            track = next(
                filter(lambda track: track.id == self._track_id, tracks.people),
                None,
            )
            # keep a sliding window of the tracks velocity
            if track is not None:
                track_vels.append((track.vel_x, track.vel_y))
                if len(track_vels) > 10:
                    track_vels.pop(0)

            if track is None:
                rospy.loginfo("Lost track of person, recovering...")
                person_trajectory = PoseArray()
                ask_back: bool = False
                self._recover_track(say=True)
                prev_track = None
                continue

            if prev_track is None:
                robot_pose: PoseStamped = self._robot_pose_in_odom()

                goal_pose = self._tf_pose(
                    PoseStamped(
                        pose=Pose(
                            position=track.pose.position,
                            orientation=robot_pose.pose.orientation,
                        ),
                        header=tracks.header,
                    ),
                    "map",
                )
                self._move_base(goal_pose)
                prev_goal = goal_pose
                last_goal_time = rospy.Time.now()
                prev_track = track

            # Check if the person is walking too fast and ask them to slow down only
            # if you haven't asked them in the last 15 seconds
            if (
                np.mean([np.linalg.norm(vel) for vel in track_vels]) > self._max_speed
                and (rospy.Time.now() - asking_time).to_sec() > self._asking_time_limit
            ):
                self._tts("Please walk slower, I am struggling to keep up", wait=False)
                asking_time = rospy.Time.now()

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
                self._waypoints.append(goal_pose)
                prev_goal = goal_pose
                prev_track = track
                last_goal_time = rospy.Time.now()
            # retry goal if it was aborted
            elif (
                self._move_base_client.get_state() in [GoalStatus.ABORTED]
                and prev_goal is not None
            ):
                rospy.logwarn("Goal was aborted, retrying")
                self._move_base(prev_goal)
                #result.waypoints.append(prev_goal)
                # self._waypoints.append(prev_goal)
            # check if the person has been static for too long
            elif (
                (
                    np.mean([np.linalg.norm(vel) for vel in track_vels])
                    < self._static_speed
                )
                and len(track_vels) == 10
                and prev_track is not None
            ):
                rospy.logwarn(
                    "Person has been static for too long, going to them and stopping"
                )
                # cancel current goal
                self._cancel_goal()

                # clear velocity buffer
                track_vels = []

                # clear waypoints
                self._waypoints = []

                if self._check_finished():
                    rospy.loginfo("Finished following person")

                    # navigate back to the original position from waypoints
                    # for waypoint in self._waypoints[::-1]:
                    #     self._move_base(waypoint, wait=True)

                    break
            rospy.loginfo("")
            rospy.loginfo(np.mean([np.linalg.norm(vel) for vel in track_vels]))
            rospy.loginfo("")
        return result
