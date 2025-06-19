# !/usr/bin/env python3
import rospy
import smach
import smach_ros
import actionlib
import math
import numpy as np
from typing import List, Tuple, Union
from math import atan2
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
from dynamic_reconfigure.srv import Reconfigure
from dynamic_reconfigure.msg import Config, BoolParameter, DoubleParameter, IntParameter
from std_srvs.srv import Empty
from lasr_vision_msgs.msg import (
    Sam2PromptArrays as PromptArrays,
    Sam2BboxWithFlag as BboxWithFlag,
    Detection3DArray,
    Detection3D,
)
from lasr_vision_msgs.srv import YoloDetection3D, YoloDetection3DRequest
import message_filters


class PersonFollowingUserData(smach.UserData):
    """UserData structure for person following state machine"""

    def __init__(self, **config_params):
        super(PersonFollowingUserData, self).__init__()

        # Sensor data - continuously updated
        self.current_image = None
        self.depth_image = None
        self.camera_info = None
        self.robot_pose = None

        # Tracking data
        self.track_id = None
        self.track_bbox = None
        self.target_list = []
        self.person_trajectory = PoseArray()
        self.newest_detection = None
        self.detection_quality_metrics = {}

        # Navigation data
        self.current_goal = None
        self.navigation_state = None
        self.distance_to_target = float('inf')
        self.path_history = []

        # Timing data
        self.last_good_detection_time = rospy.Time.now()
        self.last_movement_time = rospy.Time.now()
        self.recovery_start_time = None
        self.added_new_target_time = rospy.Time.now()
        self.look_at_point_time = rospy.Time.now()
        self.last_distance_warning_time = rospy.Time.now()
        self.last_scan_position_time = rospy.Time.now()

        # Status flags
        self.is_person_detected = False
        self.is_tracking_stable = False
        self.is_navigation_active = False
        self.recovery_mode_active = False
        self.condition_flag_state = True
        self.first_tracking_done = False

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
        self.target_boundary = 1.0
        self.new_goal_threshold_min = 0.25
        self.new_goal_threshold_max = 2.5
        self.stopping_distance = 0.75
        self.static_speed = 0.0015
        self.max_speed = 0.4
        self.max_following_distance = 2.5
        self.speak = True
        self.timeout = 0.0

        # Timeout durations (in seconds)
        self.good_detection_timeout = 5.0
        self.target_moving_timeout = 10.0
        self.distance_warning_interval = 5.0
        self.scan_position_duration = 1.5
        self.recovery_timeout = 30.0

        # Detection quality parameters
        self.max_distance_threshold = 0.9
        self.min_area_threshold = 0.01
        self.min_confidence_threshold = 0.5

        # YOLO parameters
        self.yolo_model = "yolo11n-pose.pt"
        self.yolo_confidence = 0.5

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

        # Override with provided parameters
        for key, value in config_params.items():
            if hasattr(self, key):
                setattr(self, key, value)
                rospy.loginfo(f"Set parameter {key} = {value}")
            else:
                rospy.logwarn(f"Unknown parameter: {key}")

        # Convert timeout parameters to Duration objects
        self.good_detection_timeout_duration = rospy.Duration(self.good_detection_timeout)
        self.target_moving_timeout_duration = rospy.Duration(self.target_moving_timeout)
        self.distance_warning_interval_duration = rospy.Duration(self.distance_warning_interval)
        self.scan_position_duration = rospy.Duration(self.scan_position_duration)
        self.recovery_timeout_duration = rospy.Duration(self.recovery_timeout)

    def update_parameters(self, **params):
        """Update parameters after initialization"""
        for key, value in params.items():
            if hasattr(self, key):
                setattr(self, key, value)
                rospy.loginfo(f"Updated parameter {key} = {value}")
            else:
                rospy.logwarn(f"Unknown parameter: {key}")

        # Update Duration objects
        self.good_detection_timeout_duration = rospy.Duration(self.good_detection_timeout)
        self.target_moving_timeout_duration = rospy.Duration(self.target_moving_timeout)
        self.distance_warning_interval_duration = rospy.Duration(self.distance_warning_interval)
        self.scan_position_duration = rospy.Duration(self.scan_position_duration)
        self.recovery_timeout_duration = rospy.Duration(self.recovery_timeout)


class PersonFollowingUserData(smach.UserData):
    """UserData structure for person following state machine"""

    def __init__(self, **config_params):
        super(PersonFollowingUserData, self).__init__()

        # Sensor data - continuously updated
        self.current_image = None
        self.depth_image = None
        self.camera_info = None
        self.robot_pose = None

        # Tracking data
        self.track_id = None
        self.track_bbox = None
        self.target_list = []
        self.person_trajectory = PoseArray()
        self.newest_detection = None
        self.detection_quality_metrics = {}

        # Navigation data
        self.current_goal = None
        self.navigation_state = None
        self.distance_to_target = float('inf')
        self.path_history = []

        # Timing data
        self.last_good_detection_time = rospy.Time.now()
        self.last_movement_time = rospy.Time.now()
        self.recovery_start_time = None
        self.added_new_target_time = rospy.Time.now()
        self.look_at_point_time = rospy.Time.now()
        self.last_distance_warning_time = rospy.Time.now()
        self.last_scan_position_time = rospy.Time.now()

        # Status flags
        self.is_person_detected = False
        self.is_tracking_stable = False
        self.is_navigation_active = False
        self.recovery_mode_active = False
        self.condition_flag_state = True
        self.first_tracking_done = False

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
        self.target_boundary = 1.0
        self.new_goal_threshold_min = 0.25
        self.new_goal_threshold_max = 2.5
        self.stopping_distance = 0.75
        self.static_speed = 0.0015
        self.max_speed = 0.4
        self.max_following_distance = 2.5
        self.speak = True
        self.timeout = 0.0

        # Timeout durations (in seconds)
        self.good_detection_timeout = 5.0
        self.target_moving_timeout = 10.0
        self.distance_warning_interval = 5.0
        self.scan_position_duration = 1.5
        self.recovery_timeout = 30.0

        # Detection quality parameters
        self.max_distance_threshold = 0.9
        self.min_area_threshold = 0.01
        self.min_confidence_threshold = 0.5

        # YOLO parameters
        self.yolo_model = "yolo11n-pose.pt"
        self.yolo_confidence = 0.5

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

        # Override with provided parameters
        for key, value in config_params.items():
            if hasattr(self, key):
                setattr(self, key, value)
                rospy.loginfo(f"Set parameter {key} = {value}")
            else:
                rospy.logwarn(f"Unknown parameter: {key}")

        # Convert timeout parameters to Duration objects
        self.good_detection_timeout_duration = rospy.Duration(self.good_detection_timeout)
        self.target_moving_timeout_duration = rospy.Duration(self.target_moving_timeout)
        self.distance_warning_interval_duration = rospy.Duration(self.distance_warning_interval)
        self.scan_position_duration = rospy.Duration(self.scan_position_duration)
        self.recovery_timeout_duration = rospy.Duration(self.recovery_timeout)


def _euclidean_distance(p1: Union[Pose, PoseStamped], p2: Union[Pose, PoseStamped]) -> float:
    """Calculate Euclidean distance between two poses"""
    pose1 = p1.pose if isinstance(p1, PoseStamped) else p1
    pose2 = p2.pose if isinstance(p2, PoseStamped) else p2

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


class PersonFollowingStateMachine(smach.StateMachine):
    """
    Complete person following state machine that can be used as:
    1. Standalone state machine - execute directly
    2. Sub-state machine - included in larger state machine workflows
    """

    def __init__(self, camera_name="xtion", **config_params):
        # Initialize as StateMachine with standard outcomes
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])

        # Store camera name
        self.camera = camera_name

        # Setup camera topics
        self.image_topic = f"/{self.camera}/rgb/image_raw"
        self.depth_topic = f"/{self.camera}/depth_registered/image_raw"
        self.depth_camera_info_topic = f"/{self.camera}/depth_registered/camera_info"

        # Initialize UserData with configuration parameters
        self.userdata = PersonFollowingUserData(**config_params)

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
        self.tts_client.wait_for_server()

        # Setup TF buffer and listener
        self.buffer = tf.Buffer(cache_time=rospy.Duration.from_sec(10.0))
        self.listener = tf.TransformListener(self.buffer)

        # Initialize head scan positions for recovery behavior
        self.userdata.recovery_scan_positions = []
        for angle in self.userdata.recovery_scan_angles:
            angle_rad = math.radians(angle)
            x = self.userdata.recovery_scan_distance * math.cos(angle_rad)
            y = self.userdata.recovery_scan_distance * math.sin(angle_rad)
            z = self.userdata.recovery_scan_height
            point = Point(x=x, y=y, z=z)
            self.userdata.recovery_scan_positions.append(point)

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
            config.ints.append(IntParameter(name="width", value=self.userdata.costmap_width))
            config.ints.append(IntParameter(name="height", value=self.userdata.costmap_height))
            self.dynamic_costmap(config)

            # Set maximum velocity
            config = Config()
            config.doubles.append(
                DoubleParameter(name="max_vel_x", value=self.userdata.max_speed)
            )
            self.dynamic_velocity(config)

            # Disable recovery behaviors
            config = Config()
            config.bools.append(BoolParameter(name="recovery_behavior_enabled", value=0))
            config.bools.append(BoolParameter(name="clearing_rotation_allowed", value=0))
            self.dynamic_recovery(config)

            # Set local costmap inflation radius
            config = Config()
            config.bools.append(BoolParameter(name="enabled", value=1))
            config.doubles.append(DoubleParameter(name="inflation_radius", value=self.userdata.inflation_radius))
            self.dynamic_local_costmap(config)

            # Clear existing costmaps
            rospy.ServiceProxy("/move_base/clear_costmaps", Empty)()

            rospy.sleep(1)
            rospy.loginfo("Navigation parameters configured")
        except Exception as e:
            rospy.logwarn(f"Failed to configure navigation: {e}")

        # Build the state machine structure
        self._build_state_machine()

        rospy.loginfo("Person following state machine initialized")

    def _build_state_machine(self):
        """Build the complete state machine structure"""
        with self:
            smach.StateMachine.add('INITIALIZING',
                                   InitializingState(self),
                                   transitions={'initialized': 'PERSON_DETECTION',
                                                'failed': 'failed'})

            smach.StateMachine.add('PERSON_DETECTION',
                                   PersonDetectionState(self),
                                   transitions={'person_detected': 'TRACKING_ACTIVE',
                                                'no_person_found': 'failed',
                                                'failed': 'failed'})

            smach.StateMachine.add('TRACKING_ACTIVE',
                                   TrackingActiveState(self),
                                   transitions={'navigate_to_target': 'NAVIGATION',
                                                'target_lost': 'RECOVERY_SCANNING',
                                                'following_complete': 'succeeded',
                                                'distance_warning': 'DISTANCE_WARNING',
                                                'failed': 'failed'})

            smach.StateMachine.add('NAVIGATION',
                                   NavigationState(self),
                                   transitions={'navigation_complete': 'TRACKING_ACTIVE',
                                                'target_lost': 'RECOVERY_SCANNING',
                                                'failed': 'failed'})

            smach.StateMachine.add('RECOVERY_SCANNING',
                                   RecoveryScanningState(self),
                                   transitions={'target_recovered': 'TRACKING_ACTIVE',
                                                'recovery_failed': 'failed',
                                                'failed': 'failed'})

            smach.StateMachine.add('DISTANCE_WARNING',
                                   DistanceWarningState(self),
                                   transitions={'warning_complete': 'TRACKING_ACTIVE',
                                                'failed': 'failed'})

    def execute(self, userdata=None):
        """
        Execute the state machine
        Can be called directly or as part of a larger state machine
        """
        # Initialize trajectory and timing
        self.userdata.person_trajectory.header.frame_id = "odom"
        self.userdata.start_time = rospy.Time.now()

        # Execute the state machine
        outcome = smach.StateMachine.execute(self, userdata)

        # Calculate final metrics
        self.userdata.following_duration = (rospy.Time.now() - self.userdata.start_time).to_sec()

        # Cleanup resources
        self._cleanup()

        return outcome

    def _sensor_callback(self, image: Image, depth_image: Image, depth_camera_info: CameraInfo):
        """Continuous callback for synchronized camera data"""
        self.userdata.current_image = image
        self.userdata.depth_image = depth_image
        self.userdata.camera_info = depth_camera_info

        # Update robot pose if TF is available
        if self.buffer:
            try:
                transform = self.buffer.lookup_transform(
                    "odom", "base_link", rospy.Time(0), rospy.Duration(1.0)
                )
                robot_pose = PoseStamped()
                robot_pose.header.frame_id = "odom"
                robot_pose.header.stamp = rospy.Time.now()
                robot_pose.pose.position.x = transform.transform.translation.x
                robot_pose.pose.position.y = transform.transform.translation.y
                robot_pose.pose.position.z = transform.transform.translation.z
                robot_pose.pose.orientation = transform.transform.rotation
                self.userdata.robot_pose = robot_pose
            except:
                pass  # Continue if TF lookup fails

    def _detection3d_callback(self, msg: Detection3DArray):
        """Callback for SAM2 3D detections - updates userdata"""
        for detection in msg.detections:
            if int(detection.name) == self.userdata.track_id:
                # Quality check logic
                is_good_quality = self._assess_detection_quality(detection)

                if is_reasonable_detection(detection, self.userdata.min_confidence_threshold):
                    self.userdata.newest_detection = detection

                if is_good_quality:
                    self.condition_frame_flag_pub.publish(Bool(data=True))
                    self.userdata.condition_flag_state = True
                    self.userdata.last_good_detection_time = rospy.Time.now()

                    # Transform to odom frame and add to target list
                    odom_pose = self._transform_detection_to_odom(detection)
                    if odom_pose:
                        self._update_target_list(odom_pose)
                else:
                    self.condition_frame_flag_pub.publish(Bool(data=False))
                    self.userdata.condition_flag_state = False
                break

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
        if self.userdata.camera_info:
            image_width = self.userdata.camera_info.width
            image_height = self.userdata.camera_info.height

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
                normalized_distance < self.userdata.max_distance_threshold
                and normalized_area > self.userdata.min_area_threshold
                and detection.confidence > self.userdata.min_confidence_threshold
        )

    def _transform_detection_to_odom(self, detection):
        """Transform detection point to odom frame"""
        map_pose = PoseStamped()
        map_pose.header.frame_id = "map"
        map_pose.header.stamp = rospy.Time.now()
        map_pose.pose.position = detection.point
        map_pose.pose.orientation.w = 1.0

        try:
            odom_pose = self.buffer.transform(map_pose, "odom", rospy.Duration(0.50))
            return odom_pose
        except:
            return None

    def _update_target_list(self, odom_pose):
        """Update target list with new pose"""
        if len(self.userdata.target_list) == 0:
            self.userdata.person_trajectory.poses.append(odom_pose.pose)
            self.userdata.target_list.append(odom_pose.pose)
            self.userdata.added_new_target_time = rospy.Time.now()
        else:
            prev_pose = self.userdata.target_list[-1]
            dist_to_prev = _euclidean_distance(prev_pose, odom_pose.pose)
            if (
                    self.userdata.new_goal_threshold_min
                    < dist_to_prev
                    < self.userdata.new_goal_threshold_max
            ):
                self.userdata.person_trajectory.poses.append(odom_pose.pose)
                self.userdata.target_list.append(odom_pose.pose)
                self.userdata.added_new_target_time = rospy.Time.now()

    def _cleanup(self):
        """Clean up resources after execution"""
        self.track_flag_pub.publish(Bool(data=False))
        if self.move_base_client:
            self.move_base_client.cancel_all_goals()
        rospy.loginfo("State machine cleaned up")

    def get_results(self):
        """Get execution results - useful when used as sub-state machine"""
        return {
            'distance_traveled': self.userdata.distance_traveled,
            'following_duration': self.userdata.following_duration,
            'completion_reason': self.userdata.completion_reason,
            'person_trajectory': self.userdata.person_trajectory
        }

    def _tts(self, text: str, wait: bool = False):
        """Text-to-speech with speaking flag check"""
        if self.userdata.speak:
            tts_goal = TtsGoal()
            tts_goal.rawtext.text = text
            tts_goal.rawtext.lang_id = "en_GB"
            if wait:
                self.tts_client.send_goal_and_wait(tts_goal)
            else:
                self.tts_client.send_goal(tts_goal)

    def _look_at_point(self, target_point: Point, target_frame: str = "map"):
        current_time = rospy.Time.now()
        if current_time - self.userdata.look_at_point_time < rospy.Duration(self.userdata.head_movement_interval):
            return

        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time(0)
        goal.target.header.frame_id = target_frame
        goal.target.point = target_point
        goal.pointing_frame = self.userdata.head_pointing_frame
        goal.pointing_axis.x = 0.0
        goal.pointing_axis.y = 0.0
        goal.pointing_axis.z = 1.0
        goal.min_duration = rospy.Duration(self.userdata.head_min_duration)
        goal.max_velocity = self.userdata.head_max_velocity

        self.point_head_client.send_goal(goal)
        self.userdata.look_at_point_time = current_time

    def _look_centre_point(self):
        """Look at center point"""
        point = Point(
            x=self.userdata.center_look_point[0],
            y=self.userdata.center_look_point[1],
            z=self.userdata.center_look_point[2]
        )
        self._look_at_point(point, target_frame="base_link")


# State classes remain the same but without the IdleState
class InitializingState(smach.State):
    """Initialize tracking and setup systems"""

    def __init__(self, sm_manager):
        smach.State.__init__(self, outcomes=['initialized', 'failed'])
        self.sm_manager = sm_manager

    def execute(self, userdata):
        rospy.loginfo("Initializing person following system")

        # Wait for sensor data to be available
        while not rospy.is_shutdown():
            if (userdata.current_image and
                    userdata.depth_image and
                    userdata.camera_info):
                break
            rospy.sleep(0.1)

        # Initialize tracking data
        userdata.target_list = []
        userdata.person_trajectory = PoseArray()
        userdata.person_trajectory.header.frame_id = "odom"
        userdata.last_movement_time = rospy.Time.now()
        userdata.last_good_detection_time = rospy.Time.now()
        userdata.is_tracking_stable = False

        if userdata.robot_pose:
            userdata.last_position = userdata.robot_pose

        rospy.loginfo("Initialization complete")
        return 'initialized'


class PersonDetectionState(smach.State):
    """Detect and select person to follow"""

    def __init__(self, sm_manager):
        smach.State.__init__(self, outcomes=['person_detected', 'no_person_found', 'failed'])
        self.sm_manager = sm_manager

    def execute(self, userdata):
        rospy.loginfo("Starting person detection")

        # Look forward and scan for people
        self.sm_manager._look_centre_point()
        rospy.sleep(1.0)

        # Call YOLO detection service
        req = YoloDetection3DRequest(
            image_raw=userdata.current_image,
            depth_image=userdata.depth_image,
            depth_camera_info=userdata.camera_info,
            model=userdata.yolo_model,
            confidence=userdata.yolo_confidence,
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
            return 'no_person_found'

        # Find nearest person to follow
        transform = self.sm_manager.buffer.lookup_transform(
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
            return 'no_person_found'

        # Store tracking information
        userdata.track_bbox = detected_people["xywh"][nearest_index]
        userdata.track_id = nearest_index

        rospy.loginfo(f"Selected person at distance {min_dist:.2f}m with ID {nearest_index}")

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

        # Start tracking
        self.sm_manager.track_flag_pub.publish(Bool(data=True))
        rospy.sleep(0.1)

        self.sm_manager.condition_frame_flag_pub.publish(Bool(data=True))
        userdata.condition_flag_state = True
        userdata.is_person_detected = True

        rospy.loginfo(f"Started tracking person with ID {userdata.track_id}")
        return 'person_detected'


# Other state classes remain unchanged...
class TrackingActiveState(smach.State):
    """Active tracking state - monitors target and decides on actions"""

    def __init__(self, sm_manager):
        smach.State.__init__(self, outcomes=['navigate_to_target', 'target_lost',
                                             'following_complete', 'distance_warning', 'failed'])
        self.sm_manager = sm_manager
        self.previous_target = None
        self.just_started = True

    def execute(self, userdata):
        rospy.loginfo("Active tracking mode")

        if not userdata.is_tracking_stable:
            userdata.is_tracking_stable = True
            self.sm_manager._tts("I will start to follow you.", wait=True)

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Update distance traveled
            if userdata.last_position and userdata.robot_pose:
                distance_increment = self.sm_manager._euclidean_distance(
                    userdata.last_position, userdata.robot_pose
                )
                userdata.distance_traveled += distance_increment
                userdata.last_position = userdata.robot_pose

            # Check if target is lost
            if (rospy.Time.now() - userdata.last_good_detection_time >
                    userdata.target_moving_timeout_duration):
                rospy.loginfo("Target lost - no good detection")
                return 'target_lost'

            # Check if following is complete (no movement for a while)
            if (not userdata.is_navigation_active and not self.just_started and
                    rospy.Time.now() - userdata.last_movement_time >
                    userdata.target_moving_timeout_duration):
                rospy.loginfo("Following complete - no movement detected")
                userdata.completion_reason = "ARRIVED"
                return 'following_complete'

            # Check navigation status
            if userdata.is_navigation_active and self.sm_manager.move_base_client:
                nav_state = self.sm_manager.move_base_client.get_state()
                if nav_state not in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
                    userdata.is_navigation_active = False
                    userdata.last_movement_time = rospy.Time.now()
                    rospy.loginfo("Navigation goal reached")

            # Decide if we should navigate to target
            if len(userdata.target_list) > 0 and not userdata.is_navigation_active:
                nav_decision = self._should_navigate_to_target(userdata)
                if nav_decision == 'navigate':
                    return 'navigate_to_target'
                elif nav_decision == 'distance_warning':
                    return 'distance_warning'

            # Look at detected person
            if userdata.newest_detection:
                self.sm_manager._look_at_point(
                    userdata.newest_detection.point, target_frame="map"
                )

            rate.sleep()

        return 'failed'

    def _should_navigate_to_target(self, userdata):
        """Determine if robot should navigate to target"""
        if len(userdata.target_list) == 0:
            return None

        last_pose_in_list = userdata.target_list[-1]
        target_pose = None

        # Find suitable target pose within boundary
        for i in reversed(range(len(userdata.target_list))):
            if (userdata.target_boundary <=
                    self.sm_manager._euclidean_distance(userdata.target_list[i], last_pose_in_list)):
                target_pose = userdata.target_list[i]
                break

        if target_pose is None:
            return None

        # Check if target is different enough from previous
        if (self.previous_target and
                self.sm_manager._euclidean_distance(target_pose, self.previous_target) <
                userdata.new_goal_threshold_min):
            return None

        if userdata.robot_pose:
            distance_to_target = self.sm_manager._euclidean_distance(
                userdata.robot_pose.pose, target_pose
            )

            # Check if target is too far
            if distance_to_target > userdata.max_following_distance:
                if (rospy.Time.now() - userdata.last_distance_warning_time >
                        userdata.distance_warning_interval_duration):
                    return 'distance_warning'

            # Check if we're close enough already
            if distance_to_target < userdata.stopping_distance:
                return None

        # Set current goal and proceed with navigation
        userdata.current_goal = target_pose
        self.previous_target = target_pose
        self.just_started = False

        return 'navigate'


class NavigationState(smach.State):
    """Handle navigation to target position"""

    def __init__(self, sm_manager):
        smach.State.__init__(self, outcomes=['navigation_complete', 'target_lost', 'failed'])
        self.sm_manager = sm_manager

    def execute(self, userdata):
        rospy.loginfo("Starting navigation to target")

        if not userdata.current_goal:
            return 'failed'

        # Compute goal orientation to face the person
        last_pose = userdata.target_list[-1] if userdata.target_list else userdata.current_goal
        goal_orientation = self.sm_manager._compute_face_quat(userdata.current_goal, last_pose)

        pose_with_orientation = Pose(
            position=userdata.current_goal.position,
            orientation=goal_orientation,
        )

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "odom"
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = pose_with_orientation

        # Transform to map frame for navigation
        goal_pose = self.sm_manager.buffer.transform(pose_stamped, "map", rospy.Duration(1.0))

        # Send navigation goal
        goal = MoveBaseGoal()
        goal.target_pose = goal_pose
        self.sm_manager.move_base_client.send_goal(goal)
        userdata.is_navigation_active = True

        rospy.loginfo("Navigation goal sent")

        # Monitor navigation progress
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Check if target is lost during navigation
            if (rospy.Time.now() - userdata.last_good_detection_time >
                    userdata.target_moving_timeout_duration):
                self.sm_manager.move_base_client.cancel_goal()
                userdata.is_navigation_active = False
                return 'target_lost'

            # Check navigation status
            nav_state = self.sm_manager.move_base_client.get_state()
            if nav_state not in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
                userdata.is_navigation_active = False
                userdata.last_movement_time = rospy.Time.now()

                if nav_state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Navigation completed successfully")
                    return 'navigation_complete'
                else:
                    rospy.logwarn(f"Navigation failed with status: {nav_state}")
                    return 'navigation_complete'  # Continue even if navigation failed

            rate.sleep()

        return 'failed'


class RecoveryScanningState(smach.State):
    """Recovery behavior when target is lost"""

    def __init__(self, sm_manager):
        smach.State.__init__(self, outcomes=['target_recovered', 'recovery_failed', 'failed'])
        self.sm_manager = sm_manager

    def execute(self, userdata):
        rospy.loginfo("Starting recovery scanning behavior")

        # Cancel any active navigation
        if self.sm_manager.move_base_client:
            self.sm_manager.move_base_client.cancel_all_goals()
        userdata.is_navigation_active = False

        # Initialize recovery mode
        userdata.recovery_mode_active = True
        userdata.current_scan_index = 0
        userdata.recovery_start_time = rospy.Time.now()
        userdata.last_scan_position_time = rospy.Time.now()

        self.sm_manager._tts("Let me look for you.", wait=False)

        # Start scanning
        if userdata.recovery_scan_positions:
            self.sm_manager._look_at_point(
                userdata.recovery_scan_positions[0], target_frame="base_link"
            )

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Check recovery timeout
            if (rospy.Time.now() - userdata.recovery_start_time >
                    userdata.recovery_timeout_duration):
                rospy.logwarn("Recovery behavior timed out")
                userdata.recovery_mode_active = False
                userdata.completion_reason = "LOST_PERSON"
                return 'recovery_failed'

            # Check if target is recovered
            if (rospy.Time.now() - userdata.last_good_detection_time <
                    rospy.Duration(2.0)):
                rospy.loginfo("Target recovered during scanning!")
                self.sm_manager._tts("Found you! Continuing to follow.", wait=False)
                userdata.recovery_mode_active = False
                return 'target_recovered'

            # Move to next scan position
            if (rospy.Time.now() - userdata.last_scan_position_time >
                    userdata.scan_position_duration):
                userdata.current_scan_index += 1

                if userdata.current_scan_index >= len(userdata.recovery_scan_positions):
                    userdata.current_scan_index = 0
                    rospy.loginfo("Completed one full scan cycle, starting over")

                scan_point = userdata.recovery_scan_positions[userdata.current_scan_index]
                self.sm_manager._look_at_point(scan_point, target_frame="base_link")
                userdata.last_scan_position_time = rospy.Time.now()

            rate.sleep()

        return 'failed'


class DistanceWarningState(smach.State):
    """Handle distance warning when target is too far"""

    def __init__(self, sm_manager):
        smach.State.__init__(self, outcomes=['warning_complete', 'failed'])
        self.sm_manager = sm_manager

    def execute(self, userdata):
        rospy.loginfo("Target too far - issuing distance warning")

        if userdata.robot_pose and len(userdata.target_list) > 0:
            distance = self.sm_manager._euclidean_distance(
                userdata.robot_pose.pose, userdata.target_list[-1]
            )
            rospy.loginfo(f"Target distance: {distance:.2f}m > {userdata.max_following_distance}m")

        self.sm_manager._tts("Please wait for me. You are too far away.", wait=False)
        userdata.last_distance_warning_time = rospy.Time.now()

        return 'warning_complete'


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
        person_following_sm = PersonFollowingStateMachine(
            camera_name="xtion",
            max_speed=0.5,
            stopping_distance=0.8,
            speak=True,
            recovery_timeout=25.0,
            min_confidence_threshold=0.6
        )

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
