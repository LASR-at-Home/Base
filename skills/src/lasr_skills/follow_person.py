# !/usr/bin/env python3
import copy
import rospy
import smach
import smach_ros
import actionlib
import math
import numpy as np
from typing import List, Tuple, Union
from math import atan2
from tf.transformations import quaternion_from_euler  # this is tf tool pack, not tf1
import tf2_ros as tf
from tf2_geometry_msgs import do_transform_pose
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, PoseArray
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import MarkerArray
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from nav_msgs.msg import Odometry
from dynamic_reconfigure.srv import Reconfigure
from dynamic_reconfigure.msg import Config, BoolParameter, DoubleParameter, IntParameter
from nav_msgs.srv import GetPlan, GetPlanRequest
from std_srvs.srv import Empty
from lasr_vision_msgs.msg import (
    Sam2PromptArrays as PromptArrays,
    Sam2BboxWithFlag as BboxWithFlag,
    Detection3DArray,
    Detection3D,
)
from lasr_vision_msgs.srv import YoloDetection3D, YoloDetection3DRequest
import message_filters


class PersonFollowingData:
    """UserData structure for person following state machine"""

    def __init__(self, **config_params):
        super(PersonFollowingData, self).__init__()

        # Sensor data - continuously updated
        self.current_image = None
        self.depth_image = None
        self.camera_info = None

        # Tracking data
        self.track_id = None
        self.track_bbox = None
        self.target_list = []
        self.person_trajectory = PoseArray()
        self.person_pose_stampeds = []
        self.newest_detection = None
        self.detection_quality_metrics = {}
        self.target_speed = 0.0

        # Navigation data
        self.current_goal = None
        self.navigation_state = None
        self.distance_to_target = float("inf")
        self.path_history = []

        # Timing data
        self.last_good_detection_time = rospy.Time.now()
        self.last_movement_time = rospy.Time.now()
        self.recovery_start_time = None
        self.added_new_target_time = rospy.Time.now()
        self.look_at_point_time = rospy.Time.now()
        self.last_distance_warning_time = rospy.Time.now()
        self.last_scan_position_time = rospy.Time.now()
        self.last_canceled_goal_time = rospy.Time.now()

        # Status flags
        self.is_person_detected = False
        self.is_navigation_active = False
        self.recovery_mode_active = False
        self.condition_flag_state = True
        self.first_tracking_done = False
        self.say_started = False

        # Recovery behavior data
        self.recovery_scan_positions = []
        self.current_scan_index = 0

        # Result data
        self.distance_traveled = 0.0
        self.following_duration = 0.0
        self.completion_reason = "UNKNOWN"
        self.start_time = rospy.Time.now()
        self.last_position = None

        # Configuration parameters with defaults
        self.new_goal_threshold_min = 0.35
        self.new_goal_threshold_max = 4.0
        self.stopping_distance = 2.25
        self.max_speed = 0.4
        self.max_following_distance = 4.0
        self.min_following_distance = 0.5
        self.speak = True
        self.timeout = 0.0
        self.replan_distance = 2.0

        # Timeout durations (in seconds)
        self.good_detection_timeout = 5.0
        self.target_moving_timeout = 10.0
        self.distance_warning_interval = 10.0
        self.scan_position_duration = 1.5
        self.recovery_timeout = 30.0
        self.cancel_goal_timeout = 10.0  # this should actually be big

        # Detection quality parameters
        self.max_distance_threshold = 0.9
        self.min_area_threshold = 0.01
        self.min_confidence_threshold = 0.5

        # YOLO parameters
        self.yolo_model = "yolo11n-pose.pt"
        self.yolo_confidence = 0.6

        # Head control parameters
        self.head_movement_interval = 0.35
        self.head_min_duration = 0.1
        self.head_max_velocity = 0.33
        self.head_pointing_frame = "xtion_rgb_optical_frame"

        # Recovery scan parameters
        self.recovery_scan_angles = [-60, -30, 0, 30, 60, 30, 0, -30]
        self.recovery_scan_distance = 3.0
        self.recovery_scan_height = 1.3

        # Look center point parameters
        self.center_look_point = [3.0, 0.0, 1.3]

        # Navigation parameters
        self.costmap_width = 4
        self.costmap_height = 4
        self.inflation_radius = 0.2
        self.previous_target = None
        self.last_target_position = None
        self.last_target_time = None

        # Override with provided parameters
        for key, value in config_params.items():
            if hasattr(self, key):
                setattr(self, key, value)
                rospy.loginfo(f"Set parameter {key} = {value}")
            else:
                rospy.logwarn(f"Unknown parameter: {key}")

        # Convert timeout parameters to Duration objects
        self.good_detection_timeout_duration = rospy.Duration(
            self.good_detection_timeout
        )
        self.target_moving_timeout_duration = rospy.Duration(self.target_moving_timeout)
        self.distance_warning_interval_duration = rospy.Duration(
            self.distance_warning_interval
        )
        self.scan_position_duration = rospy.Duration(self.scan_position_duration)
        self.recovery_timeout_duration = rospy.Duration(self.recovery_timeout)
        self.cancel_goal_duration = rospy.Duration(self.cancel_goal_timeout)


def _euclidean_distance(
    p1: Union[Pose, PoseStamped], p2: Union[Pose, PoseStamped]
) -> float:
    """Calculate Euclidean distance between two poses"""
    pose1 = p1.pose if isinstance(p1, PoseStamped) else p1
    pose2 = p2.pose if isinstance(p2, PoseStamped) else p2

    # rospy.loginfo(f"{np.linalg.norm(np.array([pose1.position.x, pose1.position.y])- np.array([pose2.position.x, pose2.position.y])).astype(float)}")

    return np.linalg.norm(
        np.array([pose1.position.x, pose1.position.y])
        - np.array([pose2.position.x, pose2.position.y])
    ).astype(float)


def _compute_face_quat(p1: Pose, p2: Pose) -> Quaternion:
    """Compute quaternion to face from p1 to p2"""
    dx = p2.position.x - p1.position.x
    dy = p2.position.y - p1.position.y
    theta_deg = np.degrees(atan2(dy, dx))
    x, y, z, w = R.from_euler("z", theta_deg, degrees=True).as_quat()
    return Quaternion(x, y, z, w)


class FollowPerson(smach.StateMachine):
    """
    Complete person following state machine that can be used as:
    1. Standalone state machine - execute directly
    2. Sub-state machine - included in larger state machine workflows
    """

    def __init__(self, camera_name="xtion", **config_params):
        # Initialise as StateMachine with dynamic input/output keys
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=[],
            output_keys=["start_pose", "end_pose", "location"],
        )

        # Store camera name
        self.camera = camera_name

        # Setup camera topics
        self.image_topic = f"/{self.camera}/rgb/image_raw"
        self.depth_topic = f"/{self.camera}/depth_registered/image_raw"
        self.depth_camera_info_topic = f"/{self.camera}/depth_registered/camera_info"

        # Initialise UserData with configuration parameters
        self.shared_data = PersonFollowingData(**config_params)

        # Setup synchronized camera subscribers for continuous data update
        image_sub = message_filters.Subscriber(self.image_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        depth_camera_info_sub = message_filters.Subscriber(
            self.depth_camera_info_topic, CameraInfo
        )
        ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub, depth_camera_info_sub], 10, 0.1
        )
        ts.registerCallback(self._sensor_callback)

        # Setup SAM2 publishers
        self.prompt_pub = rospy.Publisher(
            "/sam2/prompt_arrays", PromptArrays, queue_size=1
        )
        self.track_flag_pub = rospy.Publisher("/sam2/track_flag", Bool, queue_size=1)
        self.condition_frame_flag_pub = rospy.Publisher(
            "/sam2/add_conditioning_frame_flag", Bool, queue_size=1
        )

        # Setup visualization and YOLO service
        self.trajectory_pose_pub = rospy.Publisher(
            "/person_trajectory_poses", PoseArray, queue_size=1
        )

        # Setup service clients
        self.yolo = rospy.ServiceProxy("/yolo/detect3d", YoloDetection3D)
        rospy.wait_for_service("/yolo/detect3d")

        # Dynamic reconfigure services
        self.dynamic_costmap = rospy.ServiceProxy(
            "/move_base/local_costmap/set_parameters", Reconfigure
        )
        self.dynamic_velocity = rospy.ServiceProxy(
            "/move_base/PalLocalPlanner/set_parameters", Reconfigure
        )
        self.dynamic_recovery = rospy.ServiceProxy(
            "/move_base/set_parameters", Reconfigure
        )
        self.dynamic_local_costmap = rospy.ServiceProxy(
            "/move_base/local_costmap/inflation_layer/set_parameters", Reconfigure
        )

        # Add service client for global planning
        self.make_plan_service = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
        rospy.wait_for_service("/move_base/make_plan")

        # Setup action clients
        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        self.move_base_client.wait_for_server()

        self.point_head_client = actionlib.SimpleActionClient(
            "/head_controller/point_head_action", PointHeadAction
        )
        self.point_head_client.wait_for_server()

        self.tts_client = actionlib.SimpleActionClient("tts", TtsAction)
        if not self.tts_client.wait_for_server(timeout=rospy.Duration(5.0)):
            rospy.logwarn("TTS server not available after 5 seconds timeout")
            self.tts_client = False
        else:
            rospy.loginfo("TTS server connected successfully")

        # Setup TF buffer and listener
        self.tf_buffer = tf.Buffer(cache_time=rospy.Duration.from_sec(10.0))
        self.tf_listener = tf.TransformListener(self.tf_buffer)

        # temp detection:
        self.detection = None

        # Initialise head scan positions for recovery behavior
        self.shared_data.recovery_scan_positions = []
        for angle in self.shared_data.recovery_scan_angles:
            angle_rad = math.radians(angle)
            x = self.shared_data.recovery_scan_distance * math.cos(angle_rad)
            y = self.shared_data.recovery_scan_distance * math.sin(angle_rad)
            z = self.shared_data.recovery_scan_height
            point = Point(x=x, y=y, z=z)
            self.shared_data.recovery_scan_positions.append(point)

        # Setup detection subscriber
        self.detection3d_sub = rospy.Subscriber(
            "/sam2/detections_3d",
            Detection3DArray,
            self._detection3d_callback,
            queue_size=1,
        )

        try:
            # Set costmap size using userdata parameters
            config = Config()
            config.ints.append(
                IntParameter(name="width", value=self.shared_data.costmap_width)
            )
            config.ints.append(
                IntParameter(name="height", value=self.shared_data.costmap_height)
            )
            self.dynamic_costmap(config)

            # Set maximum velocity
            config = Config()
            config.doubles.append(
                DoubleParameter(name="max_vel_x", value=self.shared_data.max_speed)
            )
            self.dynamic_velocity(config)

            # Disable recovery behaviors
            config = Config()
            config.bools.append(
                BoolParameter(name="recovery_behavior_enabled", value=0)
            )
            config.bools.append(
                BoolParameter(name="clearing_rotation_allowed", value=0)
            )
            self.dynamic_recovery(config)

            # Set local costmap inflation radius
            config = Config()
            config.bools.append(BoolParameter(name="enabled", value=1))
            config.doubles.append(
                DoubleParameter(
                    name="inflation_radius", value=self.shared_data.inflation_radius
                )
            )
            self.dynamic_local_costmap(config)

            # Clear existing costmaps
            rospy.ServiceProxy("/move_base/clear_costmaps", Empty)()

            rospy.sleep(0.1)
            rospy.loginfo("Navigation parameters configured")
        except Exception as e:
            rospy.logwarn(f"Failed to configure navigation: {e}")

        # Build the state machine structure
        self._build_state_machine()

        rospy.sleep(1.0)
        rospy.loginfo("Person following state machine initialised")

    def _build_state_machine(self):
        """Build the complete state machine structure"""
        with self:
            smach.StateMachine.add(
                "Initialising",
                InitialisingState(self),
                transitions={"initialised": "PERSON_DETECTION", "failed": "failed"},
            )

            smach.StateMachine.add(
                "PERSON_DETECTION",
                PersonDetectionState(self),
                transitions={
                    "person_detected": "TRACKING_ACTIVE",
                    "no_person_found": "PERSON_DETECTION",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "TRACKING_ACTIVE",
                TrackingActiveState(self),
                transitions={
                    "navigation_started": "NAVIGATION",
                    "target_lost": "RECOVERY_SCANNING",
                    "following_complete": "succeeded",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "NAVIGATION",
                NavigationState(self),
                transitions={
                    "navigation_complete": "TRACKING_ACTIVE",
                    "target_lost": "RECOVERY_SCANNING",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "RECOVERY_SCANNING",
                RecoveryScanningState(self),
                transitions={
                    "target_recovered": "TRACKING_ACTIVE",
                    "recovery_failed": "failed",
                    "failed": "failed",
                },
            )

    def execute(self, userdata):
        """
        Execute the state machine
        Can be called directly or as part of a larger state machine
        """
        start_pose = self._get_robot_pose_in_map()
        userdata.start_pose = start_pose
        # Initialize execution-specific data
        self.shared_data.person_trajectory.header.frame_id = "map"
        self.shared_data.start_time = rospy.Time.now()
        # Execute the state machine with populated userdata
        outcome = smach.StateMachine.execute(self, self.shared_data)
        # Calculate final metrics
        self.shared_data.following_duration = (
            rospy.Time.now() - self.shared_data.start_time
        ).to_sec()
        # Cleanup resources
        self._cleanup()
        userdata.end_pose = self._get_robot_pose_in_map()
        userdata.location = start_pose.pose
        return outcome

    def _sensor_callback(
        self, image: Image, depth_image: Image, depth_camera_info: CameraInfo
    ):
        """Continuous callback for synchronized camera data"""
        self.shared_data.current_image = image
        self.shared_data.depth_image = depth_image
        self.shared_data.camera_info = depth_camera_info

    def _get_robot_pose_in_map(self) -> PoseStamped or None:
        """
        Get the current robot pose in the map coordinate frame.
        Returns:
            PoseStamped: Current robot pose in map frame
        Raises:
            tf.LookupException: If the transform cannot be found
            tf.ExtrapolationException: If the transform is too old
            tf.ConnectivityException: If the transform chain is broken
        """
        try:
            # Look up the transform from base_link to map
            transform = self.tf_buffer.lookup_transform(
                "map",  # target frame
                "base_link",  # source frame
                rospy.Time(0),  # get latest available transform
                rospy.Duration(0.1),  # timeout
            )
            # Create a PoseStamped message for the robot's current pose
            robot_pose = PoseStamped()
            robot_pose.header.frame_id = "map"
            robot_pose.header.stamp = rospy.Time.now()
            # Extract position from transform
            robot_pose.pose.position.x = transform.transform.translation.x
            robot_pose.pose.position.y = transform.transform.translation.y
            robot_pose.pose.position.z = transform.transform.translation.z
            # Extract orientation from transform
            robot_pose.pose.orientation.x = transform.transform.rotation.x
            robot_pose.pose.orientation.y = transform.transform.rotation.y
            robot_pose.pose.orientation.z = transform.transform.rotation.z
            robot_pose.pose.orientation.w = transform.transform.rotation.w
            return robot_pose
        except (
                tf.LookupException,
                tf.ExtrapolationException,
                tf.ConnectivityException,
        ) as e:
            rospy.logerr(f"Failed to get robot pose in map frame: {e}")
            return None

    def _detection3d_callback(self, msg: Detection3DArray):
        this_time = rospy.Time.now()
        """Callback for SAM2 3D detections - updates userdata"""
        if not self.shared_data.say_started:
            return

        for detection in msg.detections:
            if int(detection.name) == self.shared_data.track_id:
                # Quality check logic
                is_good_quality = self._assess_detection_quality(detection)

                if is_reasonable_detection(
                        detection, self.shared_data.min_confidence_threshold
                ):
                    self.shared_data.newest_detection = detection

                if is_good_quality:
                    for _retry in range(5):
                        try:
                            self.condition_frame_flag_pub.publish(Bool(data=True))
                            break  # success
                        except Exception as err:
                            rospy.logdebug(
                                f"Publish good-quality flag failed (attempt {_retry + 1}/5): {err}"
                            )
                            rospy.sleep(0.1)
                    else:
                        rospy.logwarn(
                            "Failed to publish good-quality flag after 5 attempts"
                        )

                    self.shared_data.condition_flag_state = True
                    self.shared_data.last_good_detection_time = this_time

                    self.detection = detection

                    # transpose into the map point
                    map_pose = PoseStamped()
                    map_pose.header.frame_id = "map"
                    map_pose.header.stamp = this_time
                    map_pose.pose.position = self.detection.point
                    map_pose.pose.orientation.w = 1.0  # Identity orientation

                    # Calculate target speed based on recent trajectory positions (dynamic window of up to 5 poses)
                    if len(self.shared_data.person_pose_stampeds) >= 2:
                        # Use the last few poses (up to 10) for speed calculation
                        window_size = min(10, len(self.shared_data.person_pose_stampeds))
                        recent_poses = self.shared_data.person_pose_stampeds[-window_size:]

                        total_distance = 0.0
                        total_time = 0.0

                        for i in range(1, len(recent_poses)):
                            # Calculate distance between consecutive poses
                            prev_pose = recent_poses[i - 1]
                            curr_pose = recent_poses[i]
                            distance = _euclidean_distance(prev_pose, curr_pose)
                            total_distance += distance

                            # Calculate time difference
                            time_diff = (curr_pose.header.stamp - prev_pose.header.stamp).to_sec()
                            total_time += time_diff

                        # Calculate average speed (m/s)
                        if total_time > 0:
                            self.shared_data.target_speed = total_distance / total_time
                            if self.shared_data.target_speed <= 0.15:
                                self.shared_data.target_speed = 0.0
                        else:
                            self.shared_data.target_speed = 0.0
                    else:
                        # Not enough trajectory data, set speed to 0
                        self.shared_data.target_speed = 0.0

                    rospy.loginfo(f"Target speed: {self.shared_data.target_speed}.")

                    if len(self.shared_data.target_list) == 0:
                        robot_pose = self._get_robot_pose_in_map()
                        # sample <num_samples> way-points inside stopping_distance
                        distance = _euclidean_distance(robot_pose, map_pose)
                        rospy.loginfo(f"Distance: {distance}")
                        if distance >= 1.75:
                            poses = self._plan_and_sample_targets(
                                robot_pose,
                                map_pose,
                                radius=self.shared_data.min_following_distance,
                                num_samples=8,
                            )
                            rospy.logwarn(f"Sampled {poses}")
                            for p in poses:
                                self._update_target_list(p, add_traj=False)
                    self._update_target_list(map_pose)
                else:
                    for _retry in range(5):
                        try:
                            self.condition_frame_flag_pub.publish(Bool(data=False))
                            break  # success
                        except Exception as err:
                            rospy.logdebug(
                                f"Publish bad-quality flag failed (attempt {_retry + 1}/5): {err}"
                            )
                            rospy.sleep(0.1)
                    else:
                        rospy.logwarn(
                            "Failed to publish bad-quality flag after 5 attempts"
                        )

                    self.shared_data.condition_flag_state = False
                break
        if len(self.shared_data.person_trajectory.poses) > 0:
            self.trajectory_pose_pub.publish(self.shared_data.person_trajectory)

    def _assess_detection_quality(self, detection):
        """Assess detection quality based on various metrics"""
        if not hasattr(detection, "xywh") or len(detection.xywh) != 4:
            return False

        xywh = detection.xywh
        box_width = xywh[2]
        box_height = xywh[3]
        box_area = box_width * box_height

        image_width = 640
        image_height = 480
        if self.shared_data.camera_info:
            image_width = self.shared_data.camera_info.width
            image_height = self.shared_data.camera_info.height

        center_x = xywh[0] + box_width / 2
        center_y = xywh[1] + box_height / 2

        image_center_x = image_width / 2
        image_center_y = image_height / 2
        normalized_distance = math.sqrt(
            ((center_x - image_center_x) / image_width) ** 2
            + ((center_y - image_center_y) / image_height) ** 2
        )

        normalized_area = box_area / (image_width * image_height)

        return (
            normalized_distance < self.shared_data.max_distance_threshold
            and normalized_area > self.shared_data.min_area_threshold
            and detection.confidence > self.shared_data.min_confidence_threshold
        )

    def _update_target_list(self, pose_stamped, add_traj=True):
        """Update target list with new pose"""
        if add_traj:
            self.shared_data.person_trajectory.poses.append(pose_stamped.pose)  # trajectory should always be added
            self.shared_data.person_pose_stampeds.append(pose_stamped)  # trajectory should always be added
        if len(self.shared_data.target_list) == 0:
            self.shared_data.target_list.append(pose_stamped.pose)
            self.shared_data.added_new_target_time = rospy.Time.now()
            rospy.loginfo(f"Adding target pose: {pose_stamped}")
            return True
        else:
            prev_pose = self.shared_data.target_list[-1]
            dist_to_prev = _euclidean_distance(prev_pose, pose_stamped.pose)
            if (
                self.shared_data.new_goal_threshold_min
                < dist_to_prev
                < self.shared_data.new_goal_threshold_max
            ):
                self.shared_data.target_list.append(pose_stamped.pose)
                self.shared_data.added_new_target_time = rospy.Time.now()
                rospy.loginfo(f"Adding target pose: {pose_stamped}")
                return True
        return False

    def _make_plan(
        self, start_pose: PoseStamped, goal_pose: PoseStamped, tolerance: float = 0.3
    ):
        """Call move_base make_plan service"""
        """Call /move_base/make_plan service (header.stamp set to 0)"""
        # Wait for current move_base goal to finish completely
        if self.move_base_client:
            nav_state = self.move_base_client.get_state()
            if nav_state in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
                rospy.loginfo(
                    "Waiting for current move_base goal to complete before planning..."
                )
                # Wait with timeout to avoid infinite blocking
                timeout_duration = rospy.Duration(10.0)  # 10 seconds timeout
                start_wait_time = rospy.Time.now()
                rate = rospy.Rate(10)  # 10 Hz check rate
                while not rospy.is_shutdown():
                    current_state = self.move_base_client.get_state()
                    # Check if navigation completed (any terminal state)
                    if current_state not in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
                        rospy.loginfo(
                            f"Move_base goal completed with state: {current_state}"
                        )
                        break
                    # Check timeout
                    if rospy.Time.now() - start_wait_time > timeout_duration:
                        rospy.logwarn(
                            "Timeout waiting for move_base goal to complete, proceeding with planning"
                        )
                        break
                    rate.sleep()

        req = GetPlanRequest()
        req.start = start_pose
        req.goal = goal_pose
        req.start.header.stamp = rospy.Time(0)
        req.goal.header.stamp = rospy.Time(0)
        req.tolerance = tolerance
        rospy.loginfo(
            f"[make_plan] start({start_pose.pose.position.x:.2f},"
            f"{start_pose.pose.position.y:.2f})  "
            f"goal({goal_pose.pose.position.x:.2f},"
            f"{goal_pose.pose.position.y:.2f})  tol={tolerance}"
        )
        return self.make_plan_service(req)

    def _plan_and_sample_targets(
            self,
            start_pose: PoseStamped,
            goal_pose: PoseStamped,
            radius: float,
            num_samples: int = 3,
            max_radius: float = 2.0,
            n_steps: int = 10,
    ):
        """
        Plan a short global path to a point that lies on the line (goal → robot),
        at progressively larger radii, and sample poses along that path.
        Args:
            start_pose (PoseStamped): Current robot pose (in the map frame).
            goal_pose  (PoseStamped): Original navigation goal (in the map frame).
            radius     (float)      : Initial distance (m) from goal to place the new target.
            num_samples(int)        : How many poses to take evenly along the path.
            max_radius (float)      : Upper bound for the fallback search (m).
            n_steps    (int)        : Number of radii to test between radius and max_radius
                                      (inclusive). Must be ≥ 2 to allow at least one retry.
        Returns:
            List[PoseStamped]: `num_samples` poses facing the original goal,
                               or an empty list if no path could be found.
        """
        rospy.logwarn("Generating candidate goals.")

        # Unpack start / goal positions once
        sx, sy = start_pose.pose.position.x, start_pose.pose.position.y
        gx, gy = goal_pose.pose.position.x, goal_pose.pose.position.y
        vec_x, vec_y = sx - gx, sy - gy  # goal → robot
        dist = math.hypot(vec_x, vec_y)
        if dist < 1e-3:
            rospy.logwarn("Robot already at goal; no path needed")
            return []

        # Iterate over a sequence of radii until a path is found
        radii = np.linspace(radius, max_radius, max(2, n_steps))
        path = []
        tried_radii = []

        for r in radii:
            # 1. Build a new target on the desired circle (distance = r)
            scale = r / dist
            target_x = gx + vec_x * scale
            target_y = gy + vec_y * scale

            new_goal = PoseStamped()
            new_goal.header.frame_id = goal_pose.header.frame_id
            new_goal.header.stamp = rospy.Time(0)  # safest for TF
            new_goal.pose.position.x = target_x
            new_goal.pose.position.y = target_y
            new_goal.pose.position.z = goal_pose.pose.position.z
            new_goal.pose.orientation = goal_pose.pose.orientation

            # 2. Ask the global planner for a path
            resp = self._make_plan(start_pose, new_goal, tolerance=0.5)
            path = resp.plan.poses
            tried_radii.append(r)

            if path:  # success!
                rospy.loginfo(f"Found path at r = {r:.2f} m")
                break  # leave the loop

        else:
            # The for-loop exhausted all radii without breaking => no path
            rospy.logwarn(
                f"Global planner returned empty path for all radii "
                f"{[f'{r:.2f}' for r in tried_radii]}"
            )
            return []

        # Evenly sample <num_samples> poses along the found path
        idxs = np.linspace(0, len(path) - 1, num_samples, dtype=int)
        sampled = [copy.deepcopy(path[i]) for i in idxs]

        # Rotate every pose to face the original goal
        for p in sampled:
            dx, dy = gx - p.pose.position.x, gy - p.pose.position.y
            yaw = math.atan2(dy, dx)
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
            p.pose.orientation.x, p.pose.orientation.y = qx, qy
            p.pose.orientation.z, p.pose.orientation.w = qz, qw

        rospy.logwarn("Candidate goals generated.")
        return sampled

    def _cleanup(self):
        """Clean up resources after execution"""
        self.track_flag_pub.publish(Bool(data=False))
        if self.move_base_client:
            self.move_base_client.cancel_all_goals()
        rospy.loginfo("State machine cleaned up")

    def get_results(self):
        """Get execution results - useful when used as sub-state machine"""
        return {
            "distance_traveled": self.shared_data.distance_traveled,
            "following_duration": self.shared_data.following_duration,
            "completion_reason": self.shared_data.completion_reason,
            "person_trajectory": self.shared_data.person_trajectory,
        }

    def _tts(self, text: str, wait: bool = False):
        """Text-to-speech with speaking flag check"""
        rospy.logwarn(f"You are saying '{text}'.")
        if not self.tts_client:
            return
        if self.shared_data.speak:
            tts_goal = TtsGoal()
            tts_goal.rawtext.text = text
            tts_goal.rawtext.lang_id = "en_GB"
            if wait:
                self.tts_client.send_goal_and_wait(tts_goal)
            else:
                self.tts_client.send_goal(tts_goal)
        rospy.loginfo(f"Saying '{text}'")

    def _look_at_point(self, target_point: Point, target_frame: str = "map"):
        current_time = rospy.Time.now()
        if current_time - self.shared_data.look_at_point_time < rospy.Duration(
            self.shared_data.head_movement_interval
        ):
            return

        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time(0)
        goal.target.header.frame_id = target_frame
        goal.target.point = target_point
        goal.pointing_frame = self.shared_data.head_pointing_frame
        goal.pointing_axis.x = 0.0
        goal.pointing_axis.y = 0.0
        goal.pointing_axis.z = 1.0
        goal.min_duration = rospy.Duration(self.shared_data.head_min_duration)
        goal.max_velocity = self.shared_data.head_max_velocity

        self.point_head_client.send_goal(goal)
        self.shared_data.look_at_point_time = current_time

    def _look_centre_point(self):
        """Look at center point"""
        point = Point(
            x=self.shared_data.center_look_point[0],
            y=self.shared_data.center_look_point[1],
            z=self.shared_data.center_look_point[2],
        )
        self._look_at_point(point, target_frame="base_link")

    def _look_down_point(self, target_point: Point = None, target_frame: str = "map"):
        """Look down towards the target direction or center if no target provided."""
        if target_point is not None:
            # Transform target point to base_link frame to get direction
            try:
                target_pose = PoseStamped()
                target_pose.header.frame_id = target_frame
                target_pose.header.stamp = rospy.Time.now()
                target_pose.pose.position = target_point
                target_pose.pose.orientation.w = 1.0

                # Transform to base_link frame
                target_in_base = self._tf_pose(target_pose, "base_link")

                # Calculate direction towards target but look down
                x_dir = target_in_base.pose.position.x
                y_dir = target_in_base.pose.position.y

                # Normalize the direction and set distance
                distance = 3.0
                if x_dir != 0 or y_dir != 0:
                    norm = math.sqrt(x_dir**2 + y_dir**2)
                    x_dir = (x_dir / norm) * distance
                    y_dir = (y_dir / norm) * distance
                else:
                    # Default to forward if target is directly above/below
                    x_dir = distance
                    y_dir = 0.0

                # Look down towards the target direction
                point = Point(x=x_dir, y=y_dir, z=0.5)

            except Exception as e:
                rospy.logwarn(f"Failed to transform target point: {e}")
                # Fall back to center point
                point = Point(x=3.0, y=0.0, z=0.5)
        else:
            # Default center point when no target provided
            point = Point(x=3.0, y=0.0, z=0.5)

        self._look_at_point(point, target_frame="base_link")


# State classes remain the same but without the IdleState
class InitialisingState(smach.State):
    """Initialise tracking and setup systems"""

    def __init__(self, sm_manager):
        self.sm_manager = sm_manager
        smach.State.__init__(
            self,
            outcomes=["initialised", "failed"],
            input_keys=[],
            output_keys=[],
        )

    def execute(self, userdata):
        rospy.loginfo("Initialising person following system")

        # Initialise tracking data
        self.sm_manager.shared_data.target_list = []
        person_trajectory = PoseArray()
        person_trajectory.header.frame_id = "map"
        self.sm_manager.shared_data.person_trajectory = person_trajectory
        self.sm_manager.shared_data.last_movement_time = rospy.Time.now()
        self.sm_manager.shared_data.last_good_detection_time = rospy.Time.now()

        robot_pose = self.sm_manager._get_robot_pose_in_map()
        if robot_pose:
            self.sm_manager.shared_data.last_position = robot_pose

        rospy.loginfo("Initialisation complete")
        return "initialised"


class PersonDetectionState(smach.State):
    """Detect and select person to follow"""

    def __init__(self, sm_manager):
        self.sm_manager = sm_manager
        smach.State.__init__(
            self,
            outcomes=["person_detected", "no_person_found", "failed"],
            input_keys=[],
            output_keys=[],
        )

    def execute(self, userdata):
        rospy.loginfo("Starting person detection")

        # Look forward and scan for people
        self.sm_manager._look_centre_point()
        self.sm_manager._tts("Please wait for me to detect you.", wait=True)

        # Wait for sensor data to be available
        while not rospy.is_shutdown():
            if self.sm_manager.shared_data.current_image and self.sm_manager.shared_data.depth_image and self.sm_manager.shared_data.camera_info:
                break
            rospy.sleep(0.1)

        # Call YOLO detection service
        req = YoloDetection3DRequest(
            image_raw=self.sm_manager.shared_data.current_image,
            depth_image=self.sm_manager.shared_data.depth_image,
            depth_camera_info=self.sm_manager.shared_data.camera_info,
            model=self.sm_manager.shared_data.yolo_model,
            confidence=self.sm_manager.shared_data.yolo_confidence,
            target_frame="map",
        )

        response = self.sm_manager.yolo(req)
        detections = response.detected_objects

        detected_people = {"xywh": [], "point": []}
        for detection in detections:
            detected_people["point"].append(detection.point)
            detected_people["xywh"].append(detection.xywh)

        if not detected_people["point"]:
            rospy.logwarn("No people detected for tracking")
            return "no_person_found"

        # Find nearest person to follow
        transform = self.sm_manager.tf_buffer.lookup_transform(
            "map", "base_link", rospy.Time(0), rospy.Duration(1.0)
        )
        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        min_dist = float("inf")
        nearest_index = -1

        for i, point in enumerate(detected_people["point"]):
            dx = point.x - robot_x
            dy = point.y - robot_y
            dist = math.hypot(dx, dy)

            if dist < min_dist:
                min_dist = dist
                nearest_index = i

        if nearest_index == -1:
            return "no_person_found"

        # Store tracking information
        self.sm_manager.shared_data.track_bbox = detected_people["xywh"][nearest_index]
        self.sm_manager.shared_data.track_id = nearest_index

        rospy.loginfo(
            f"Selected person at distance {min_dist:.2f}m with ID {nearest_index}"
        )

        # Setup SAM2 tracking
        bbox_list = []
        for i, bbox in enumerate(detected_people["xywh"]):
            bbox_msg = BboxWithFlag()
            bbox_msg.obj_id = i
            bbox_msg.reset = False
            bbox_msg.clear_old_points = True
            bbox_msg.xywh = bbox
            bbox_list.append(bbox_msg)

        prompt_msg = PromptArrays()
        prompt_msg.bbox_array = bbox_list
        prompt_msg.point_array = []
        prompt_msg.reset = True
        self.sm_manager.prompt_pub.publish(prompt_msg)
        rospy.sleep(0.5)
        self.sm_manager.prompt_pub.publish(prompt_msg)
        rospy.sleep(0.5)

        # Start tracking
        self.sm_manager.track_flag_pub.publish(Bool(data=True))
        rospy.sleep(0.1)

        self.sm_manager.condition_frame_flag_pub.publish(Bool(data=True))
        self.sm_manager.shared_data.condition_flag_state = True
        self.sm_manager.shared_data.is_person_detected = True

        rospy.loginfo(f"Started tracking person with ID {nearest_index}")
        return "person_detected"


class TrackingActiveState(smach.State):
    """Active tracking state - monitors target, sends navigation goals, and manages distance warnings"""
    def __init__(self, sm_manager):
        self.sm_manager = sm_manager
        smach.State.__init__(
            self,
            outcomes=[
                "navigation_started",
                "target_lost",
                "following_complete",
                "failed",
            ],
            input_keys=[],
            output_keys=[],
        )
        self.just_started = True

    def execute(self, userdata):
        rospy.loginfo("Active tracking mode")

        if not self.sm_manager.shared_data.first_tracking_done:
            self.sm_manager._tts("I will start to follow you.", wait=True)
            self.sm_manager.shared_data.say_started = True

        rate = rospy.Rate(15)

        while not rospy.is_shutdown():
            # Check if following is complete (no movement for a while)
            if (
                    self.sm_manager.shared_data.first_tracking_done and
                    rospy.Time.now() - self.sm_manager.shared_data.last_movement_time
                    > self.sm_manager.shared_data.target_moving_timeout_duration * 1.0
            ):
                rospy.loginfo("Following complete - force to stop by timeout.")
                self.sm_manager._tts(
                    "Have we arrived? I will stop following.", wait=False
                )
                self.sm_manager.shared_data.completion_reason = "ARRIVED"
                return "following_complete"

            # Look at detected person
            if self.sm_manager.shared_data.newest_detection:
                self.sm_manager._look_at_point(
                    self.sm_manager.shared_data.newest_detection.point, target_frame="map"
                )

            # Update distance traveled
            robot_pose = self.sm_manager._get_robot_pose_in_map()
            if self.sm_manager.shared_data.last_position and robot_pose:
                distance_increment = _euclidean_distance(
                    self.sm_manager.shared_data.last_position, robot_pose
                )
                self.sm_manager.shared_data.distance_traveled += distance_increment
                self.sm_manager.shared_data.last_position = robot_pose

            # Check if target is lost
            if (
                    rospy.Time.now() - self.sm_manager.shared_data.last_good_detection_time
                    > self.sm_manager.shared_data.target_moving_timeout_duration
            ):
                rospy.loginfo("Target lost - no good detection")
                return "target_lost"

            # Check if navigation is currently active
            if self.sm_manager.move_base_client:
                nav_state = self.sm_manager.move_base_client.get_state()
                nav_active = nav_state in [GoalStatus.PENDING, GoalStatus.ACTIVE]

                # If navigation is active, we should be in NAVIGATION state
                if nav_active:
                    rospy.loginfo(
                        f"Navigation active (state: {nav_state}), transitioning to NAVIGATION state"
                    )
                    return "navigation_started"
            else:
                nav_active = False

            # Handle distance warning and navigation decision
            if len(self.sm_manager.shared_data.target_list) > 0:
                rospy.loginfo(
                    f"Processing target list with {len(self.sm_manager.shared_data.target_list)} poses"
                )

                # Find suitable target pose within boundary using dynamic stopping distance
                target_pose = None
                last_pose_in_list = self.sm_manager.shared_data.target_list[-1]

                target_speed = self.sm_manager.shared_data.target_speed
                dynamic_stopping_distance = max(
                    self.sm_manager.shared_data.min_following_distance, self.sm_manager.shared_data.stopping_distance - abs(target_speed * self.sm_manager.shared_data.stopping_distance * 1.25)
                )
                for i in reversed(range(len(self.sm_manager.shared_data.target_list))):
                    distance_to_last = _euclidean_distance(
                        self.sm_manager.shared_data.target_list[i], last_pose_in_list
                    )
                    rospy.loginfo(
                        f"Checking pose {i}: distance to latest pose = {distance_to_last:.2f}m "
                        f"(dynamic threshold: {dynamic_stopping_distance:.2f}m, based on target speed: {target_speed:.2f}m)"
                    )

                    if abs(dynamic_stopping_distance - distance_to_last) <= 0.25:
                        target_pose = self.sm_manager.shared_data.target_list[i]
                        rospy.loginfo(
                            f"Selected target pose at index {i} (dynamic distance threshold met)"
                        )
                        break

                if target_pose is None:
                    rospy.loginfo(
                        "No suitable target pose found - all poses too close to latest position"
                    )
                else:
                    rospy.loginfo("Evaluating navigation decision criteria")

                    # Check if target is different enough from previous
                    if (
                            self.sm_manager.shared_data.previous_target
                            and self.sm_manager.shared_data.previous_target != self.sm_manager.shared_data.current_goal
                    ):
                        distance_to_previous = _euclidean_distance(
                            target_pose, self.sm_manager.shared_data.previous_target
                        )
                        rospy.loginfo(
                            f"Distance to previous target: {distance_to_previous:.2f}m "
                            f"(threshold: {self.sm_manager.shared_data.new_goal_threshold_min:.2f}m)"
                        )

                        if distance_to_previous < self.sm_manager.shared_data.new_goal_threshold_min:
                            rospy.loginfo(
                                "Target too similar to previous - skipping navigation"
                            )
                            target_pose = None
                    else:
                        rospy.loginfo(
                            "No previous target - proceeding with navigation evaluation"
                        )

                    # Check robot distance if we have a valid target
                    robot_pose = self.sm_manager._get_robot_pose_in_map()
                    if target_pose is not None and robot_pose:
                        distance_to_target = _euclidean_distance(
                            robot_pose.pose, target_pose
                        )
                        rospy.loginfo(
                            f"Robot distance to target: {distance_to_target:.2f}m "
                            f"(min following distance: {self.sm_manager.shared_data.min_following_distance:.2f}m, "
                            f"max following distance: {self.sm_manager.shared_data.max_following_distance:.2f}m)"
                        )

                        # Only navigate if we're not already close enough
                        if distance_to_target >= self.sm_manager.shared_data.min_following_distance:
                            rospy.loginfo(
                                "Distance check passed - sending navigation goal"
                            )
                            # Send navigation goal directly here
                            if self._send_navigation_goal(
                                    self.sm_manager.shared_data, target_pose, last_pose_in_list
                            ):
                                rospy.loginfo(
                                    "Navigation goal sent successfully, transitioning to NAVIGATION state"
                                )
                                return "navigation_started"
                            else:
                                rospy.logwarn("Failed to send navigation goal")
                        else:
                            rospy.loginfo(
                                "Robot already close enough to target - no navigation needed"
                            )

                    elif target_pose is not None:
                        rospy.loginfo(
                            "No robot pose available - cannot evaluate navigation distance"
                        )
            else:
                rospy.loginfo("Target list empty - no navigation decision possible")

            # Check if following is complete (no movement for a while)
            if (
                    not nav_active
                    and not self.just_started
                    and rospy.Time.now() - self.sm_manager.shared_data.last_movement_time
                    > self.sm_manager.shared_data.target_moving_timeout_duration
                    and rospy.Time.now() - self.sm_manager.shared_data.added_new_target_time
                    > self.sm_manager.shared_data.target_moving_timeout_duration
            ):
                rospy.loginfo("Following complete - no movement detected")
                self.sm_manager._tts(
                    "Have we arrived? I will stop following.", wait=False
                )
                self.sm_manager.shared_data.completion_reason = "ARRIVED"
                return "following_complete"

            rate.sleep()

        return "failed"

    def _send_navigation_goal(self, userdata, target_pose, orientation_reference_pose):
        """Send navigation goal to move_base and return success status"""
        self.sm_manager.shared_data.first_tracking_done = True
        try:
            # Compute goal orientation to face the person
            goal_orientation = _compute_face_quat(
                target_pose, orientation_reference_pose
            )

            pose_with_orientation = Pose(
                position=target_pose.position,
                orientation=goal_orientation,
            )

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose = pose_with_orientation

            # Transform to map frame for navigation
            goal_pose = self.sm_manager.tf_buffer.transform(
                pose_stamped, "map", rospy.Duration(1.0)
            )

            # Send navigation goal
            goal = MoveBaseGoal()
            goal.target_pose = goal_pose
            self.sm_manager.move_base_client.send_goal(goal)

            # Update tracking state
            self.sm_manager.shared_data.current_goal = target_pose
            self.sm_manager.shared_data.previous_target = target_pose
            self.just_started = False

            rospy.loginfo("Navigation goal sent successfully")
            return True

        except Exception as e:
            rospy.logerr(f"Failed to send navigation goal: {e}")
            return False


class NavigationState(smach.State):
    """Monitor navigation progress, handle head tracking, and detect completion/failure"""
    def __init__(self, sm_manager):
        self.sm_manager = sm_manager
        smach.State.__init__(
            self,
            outcomes=["navigation_complete", "target_lost", "failed"],
            input_keys=[],
            output_keys=[],
        )

    def execute(self, userdata):
        rospy.loginfo("Monitoring navigation progress")
        self.sm_manager.shared_data.last_canceled_goal_time = rospy.Time.now()

        start_robot_pose = self.sm_manager._get_robot_pose_in_map()

        # Verify that navigation is indeed active
        if not self._is_navigation_active() and self.sm_manager.shared_data.first_tracking_done:
            rospy.logwarn("Entered NAVIGATION state but no active navigation found")
            return "navigation_complete"

        # Monitor navigation progress
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            # Keep looking at detected person during navigation
            if self.sm_manager.shared_data.newest_detection:
                self.sm_manager._look_at_point(
                    self.sm_manager.shared_data.newest_detection.point, target_frame="map"
                )

            robot_pose = self.sm_manager._get_robot_pose_in_map()

            current_goal = self.sm_manager.shared_data.current_goal
            if len(self.sm_manager.shared_data.target_list) > 0 and robot_pose:
                newest_target = self.sm_manager.shared_data.target_list[-1]
                # Check if current target has deviated too much from the real target
                # if (
                #     rospy.Time.now() - self.sm_manager.shared_data.last_canceled_goal_time
                #     > self.sm_manager.shared_data.cancel_goal_duration
                #     and current_goal
                # ):
                #     deviation_distance = _euclidean_distance(
                #         current_goal, newest_target
                #     )
                #     rospy.logdebug(
                #         f"Target deviation: {deviation_distance:.2f}m vs threshold: {self.sm_manager.shared_data.replan_distance:.2f}m"
                #     )

                    # if deviation_distance > self.sm_manager.shared_data.replan_distance:
                    #     rospy.loginfo(
                    #         f"Target deviated too much ({deviation_distance:.2f}m), canceling current navigation"
                    #     )
                    #     # Cancel current navigation goal
                    #     self.sm_manager.shared_data.last_canceled_goal_time = rospy.Time.now()
                    #     self.sm_manager.move_base_client.cancel_goal()
                    #     self.sm_manager.shared_data.previous_target = None
                    #     rospy.loginfo("Current navigation goal canceled")
                    #     self.sm_manager.shared_data.target_list = (
                    #         []
                    #     )  # refresh the target list by replaning (will be trigured automatically)
                    #     self.sm_manager.shared_data.person_trajectory = PoseArray()
                    #     self.sm_manager.shared_data.person_trajectory.header.frame_id = "map"
                    #     self.sync_self.sm_manager.shared_data_to_manager(
                    #         self.sm_manager.shared_data
                    #     )  # refresh self.shared_data of parent state machine
                    #     # Send a rotation-only goal to face the real target
                    #     if self._send_face_target_goal(self.sm_manager.shared_data, newest_target):
                    #         rospy.loginfo(
                    #             "Face-target goal sent, staying in navigation state"
                    #         )
                    #         # Stay in navigation state to monitor the rotation
                    #     else:
                    #         rospy.logwarn("Failed to send face-target goal")
                    #         self.sync_self.sm_manager.shared_data_to_manager(self.sm_manager.shared_data)
                    #         return "navigation_complete"
                    #
                    #     # Update current goal to the new target
                    #     self.sm_manager.shared_data.current_goal = newest_target
                    #     continue

                # Issue distance warning if target is too far
                distance_to_target = _euclidean_distance(
                    current_goal, newest_target
                )
                if distance_to_target > self.sm_manager.shared_data.max_following_distance and (
                    rospy.Time.now() - self.sm_manager.shared_data.last_distance_warning_time
                    > self.sm_manager.shared_data.distance_warning_interval_duration
                ):
                    rospy.loginfo(
                        f"Issuing distance warning - target too far: {distance_to_target:.2f}m"
                    )
                    self.sm_manager._tts(
                        "Please wait for me. You are too far away.", wait=False
                    )
                    self.sm_manager.shared_data.last_distance_warning_time = rospy.Time.now()

            # Check navigation status
            nav_state = self.sm_manager.move_base_client.get_state()
            if nav_state not in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
                # Navigation has completed (successfully or not)
                if nav_state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Navigation completed successfully")
                else:
                    rospy.logwarn(f"Navigation ended with status: {nav_state}")

                end_robot_pose = robot_pose
                movement = _euclidean_distance(start_robot_pose.pose, end_robot_pose.pose)

                # Update movement time continuously during navigation
                if movement >= 0.01:
                    self.sm_manager.shared_data.last_movement_time = rospy.Time.now()
                    self.sm_manager.shared_data.added_new_target_time = rospy.Time.now()

                # Always return to tracking active, even if navigation failed
                return "navigation_complete"

            rate.sleep()

        return "failed"

    def _send_face_target_goal(self, userdata, target_pose):
        """Send a rotation-only goal to face the target. Robot stays at current position but rotates."""
        try:
            robot_pose = self.sm_manager._get_robot_pose_in_map()
            if not robot_pose:
                rospy.logwarn("No robot pose available for face-target goal")
                return False

            # Create goal at current robot position but with orientation facing the target
            goal_orientation = _compute_face_quat(robot_pose.pose, target_pose)

            pose_with_orientation = Pose(
                position=robot_pose.pose.position,  # Stay at current position
                orientation=goal_orientation,  # Face the target
            )

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose = pose_with_orientation

            # Transform to map frame for navigation
            goal_pose = self.sm_manager.tf_buffer.transform(
                pose_stamped, "map", rospy.Duration(1.0)
            )

            # Send navigation goal (rotation-only)
            goal = MoveBaseGoal()
            goal.target_pose = goal_pose
            self.sm_manager.move_base_client.send_goal(goal)

            rospy.loginfo("Face-target goal sent successfully")
            return True

        except Exception as e:
            rospy.logerr(f"Failed to send face-target goal: {e}")
            return False

    def _is_navigation_active(self):
        """Check if navigation is currently active based on move_base client state"""
        if not self.sm_manager.move_base_client:
            return False
        nav_state = self.sm_manager.move_base_client.get_state()
        return nav_state in [GoalStatus.PENDING, GoalStatus.ACTIVE]


class RecoveryScanningState(smach.State):
    """Recovery behavior when target is lost"""

    def __init__(self, sm_manager):
        smach.State.__init__(
            self,
            outcomes=["target_recovered", "recovery_failed", "failed"],
            input_keys=[],
            output_keys=[],
        )
        self.sm_manager = sm_manager

    def execute(self, userdata):
        rospy.loginfo("Starting recovery scanning behavior")

        # Cancel any active navigation
        if self.sm_manager.move_base_client:
            self.sm_manager.move_base_client.cancel_all_goals()
        self.sm_manager.shared_data.is_navigation_active = False

        # Initialise recovery mode
        self.sm_manager.shared_data.recovery_mode_active = True
        self.sm_manager.shared_data.current_scan_index = 0
        self.sm_manager.shared_data.recovery_start_time = rospy.Time.now()
        self.sm_manager.shared_data.last_scan_position_time = rospy.Time.now()

        self.sm_manager._tts("Let me look for you.", wait=False)

        # Start scanning
        if self.sm_manager.shared_data.recovery_scan_positions:
            self.sm_manager._look_at_point(
                self.sm_manager.shared_data.recovery_scan_positions[0], target_frame="base_link"
            )

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Check recovery timeout
            if (
                rospy.Time.now() - self.sm_manager.shared_data.recovery_start_time
                > self.sm_manager.shared_data.recovery_timeout_duration
            ):
                rospy.logwarn("Recovery behavior timed out")
                self.sm_manager.shared_data.recovery_mode_active = False
                self.sm_manager.shared_data.completion_reason = "LOST_PERSON"
                return "recovery_failed"

            # Check if target is recovered
            if rospy.Time.now() - self.sm_manager.shared_data.last_good_detection_time < rospy.Duration(
                2.0
            ):
                rospy.loginfo("Target recovered during scanning!")
                self.sm_manager._tts("Found you! Continuing to follow.", wait=False)
                self.sm_manager.shared_data.recovery_mode_active = False
                self.sm_manager.shared_data.last_movement_time = rospy.Time.now()
                self.sm_manager.shared_data.added_new_target_time = rospy.Time.now()
                # self.sm_manager.shared_data.just_started = True  # not sure if this is really needed.
                return "target_recovered"

            # Move to next scan position
            if (
                rospy.Time.now() - self.sm_manager.shared_data.last_scan_position_time
                > self.sm_manager.shared_data.scan_position_duration
            ):
                self.sm_manager.shared_data.current_scan_index += 1

                if self.sm_manager.shared_data.current_scan_index >= len(self.sm_manager.shared_data.recovery_scan_positions):
                    self.sm_manager.shared_data.current_scan_index = 0
                    rospy.loginfo("Completed one full scan cycle, starting over")

                scan_point = self.sm_manager.shared_data.recovery_scan_positions[
                    self.sm_manager.shared_data.current_scan_index
                ]
                self.sm_manager._look_at_point(scan_point, target_frame="base_link")
                self.sm_manager.shared_data.last_scan_position_time = rospy.Time.now()

            rate.sleep()

        return "failed"


def is_reasonable_detection(detection, min_confidence_threshold=0.5):
    """Check if detection has reasonable properties"""
    if not hasattr(detection, "xywh") or len(detection.xywh) != 4:
        return False

    xywh = detection.xywh
    box_width = xywh[2]
    box_height = xywh[3]
    aspect_ratio = box_width / max(box_height, 1)

    return detection.confidence > min_confidence_threshold and 0.3 < aspect_ratio < 3.0


def main():
    """Main function for standalone execution"""
    rospy.init_node("person_following_statemachine")

    try:
        # Create the state machine with custom parameters
        person_following_sm = FollowPerson()

        # Execute the state machine
        rospy.loginfo("Starting person following state machine")
        outcome = person_following_sm.execute()

        # Get results
        results = person_following_sm.get_results()
        rospy.loginfo(f"Person following completed with outcome: {outcome}")
        rospy.loginfo(f"Results: {results}")

    except rospy.ROSInterruptException:
        rospy.loginfo("Person following interrupted")


if __name__ == "__main__":
    main()
