#!/usr/bin/env python3
import rospy
import math
import time
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Pose,
    PoseStamped,
    Quaternion,
    PoseArray,
    Point,
)
from typing import Union, List, Tuple, Optional
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import PointCloud2
import message_filters
from dynamic_reconfigure.srv import Reconfigure
from dynamic_reconfigure.msg import Config, IntParameter, DoubleParameter, BoolParameter
from std_srvs.srv import Empty
from pal_interaction_msgs.msg import TtsGoal, TtsAction
import tf2_ros as tf
import tf2_geometry_msgs  # type: ignore
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from lasr_vision_msgs.msg import (
    Sam2PromptArrays as PromptArrays,
    Sam2BboxWithFlag as BboxWithFlag,
    Detection3DArray,
    Detection3D,
)
from lasr_vision_msgs.srv import YoloDetection3D, YoloDetection3DRequest
from math import atan2
import numpy as np
from scipy.spatial.transform import Rotation as R
import actionlib
from control_msgs.msg import PointHeadAction, PointHeadGoal
from lasr_person_following.srv import FollowPersonSrv, FollowPersonSrvResponse

MAX_VISION_DIST: float = 5.0


class PersonFollowerService:
    """ROS service for person following using SAM2 and YOLO detection."""

    def __init__(self):
        # Initialize ROS node
        rospy.init_node("person_following_service")

        # Create service
        self.service = rospy.Service(
            "follow_person", FollowPersonSrv, self.handle_follow_request
        )

        # Get camera name
        self.camera = rospy.get_param("~camera", "xtion")

        # Setup camera topics
        self.image_topic = f"/{self.camera}/rgb/image_raw"
        self.depth_topic = f"/{self.camera}/depth_registered/image_raw"
        self.depth_camera_info_topic = f"/{self.camera}/depth_registered/camera_info"

        # Initialize sensor data storage
        self.image, self.depth_image, self.depth_camera_info = None, None, None

        # Setup synchronized camera subscribers
        image_sub = message_filters.Subscriber(self.image_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        depth_camera_info_sub = message_filters.Subscriber(
            self.depth_camera_info_topic, CameraInfo
        )
        ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub, depth_camera_info_sub], 10, 0.1
        )
        ts.registerCallback(self.image_callback)

        # Setup SAM2 publishers and subscribers
        self.prompt_pub = rospy.Publisher(
            "/sam2/prompt_arrays", PromptArrays, queue_size=1
        )
        self.track_flag_pub = rospy.Publisher("/sam2/track_flag", Bool, queue_size=1)

        # Setup SAM2 conditioning frame control
        self.condition_frame_flag_pub = rospy.Publisher(
            "/sam2/add_conditioning_frame_flag", Bool, queue_size=1
        )

        # Setup YOLO service client
        self.yolo = rospy.ServiceProxy("/yolo/detect3d", YoloDetection3D)
        self.yolo.wait_for_service()

        # Setup navigation client
        self._move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        if not self._move_base_client.wait_for_server(rospy.Duration.from_sec(10.0)):
            rospy.logwarn("Move base client not available")

        # Setup TF buffer and listener
        self._buffer = tf.Buffer(cache_time=rospy.Duration.from_sec(10.0))
        self._listener = tf.TransformListener(self._buffer)

        # Setup head pointing client
        self._point_head_client = actionlib.SimpleActionClient(
            "/head_controller/point_head_action", PointHeadAction
        )
        if not self._point_head_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logwarn("Head pointing action server not available.")

        # Setup dynamic reconfigure service proxies
        self._dynamic_costmap = rospy.ServiceProxy(
            "/move_base/local_costmap/set_parameters", Reconfigure
        )
        self._dynamic_velocity = rospy.ServiceProxy(
            "/move_base/PalLocalPlanner/set_parameters", Reconfigure
        )
        self._dynamic_recovery = rospy.ServiceProxy(
            "/move_base/set_parameters", Reconfigure
        )
        self._dynamic_local_costmap = rospy.ServiceProxy(
            "/move_base/local_costmap/inflation_layer/set_parameters", Reconfigure
        )

        # Wait for navigation services
        rospy.wait_for_service("/move_base/set_parameters")
        rospy.wait_for_service("/move_base/local_costmap/set_parameters")
        rospy.wait_for_service("/move_base/PalLocalPlanner/set_parameters")
        rospy.wait_for_service(
            "/move_base/local_costmap/inflation_layer/set_parameters"
        )
        rospy.wait_for_service("/move_base/clear_costmaps")

        # Setup TTS client
        self._tts_client = actionlib.SimpleActionClient("tts", TtsAction)
        self._tts_client_available = self._tts_client.wait_for_server(
            rospy.Duration.from_sec(10.0)
        )
        if not self._tts_client_available:
            rospy.logwarn("TTS client not available")

        # Initialize recovery scan positions
        self.recovery_scan_positions = []
        self._initialize_recovery_positions()

        rospy.loginfo("Person following service initialized and ready")

    def handle_follow_request(self, req):
        """
        Handle incoming service requests to follow a person

        Args:
            req: FollowPersonSrv request with parameters

        Returns:
            FollowPersonSrvResponse: Result of the following operation
        """
        # Create a follower with the requested parameters
        follower = PersonFollower(
            max_speed=req.max_speed,
            stopping_distance=req.stopping_distance,
            max_following_distance=req.max_following_distance,
            speak=req.speak,
            timeout=req.timeout,
            # Pass necessary clients and publishers
            move_base_client=self._move_base_client,
            point_head_client=self._point_head_client,
            tts_client=self._tts_client,
            prompt_pub=self.prompt_pub,
            track_flag_pub=self.track_flag_pub,
            condition_frame_flag_pub=self.condition_frame_flag_pub,
            yolo=self.yolo,
            buffer=self._buffer,
            dynamic_costmap=self._dynamic_costmap,
            dynamic_velocity=self._dynamic_velocity,
            dynamic_recovery=self._dynamic_recovery,
            dynamic_local_costmap=self._dynamic_local_costmap,
            recovery_scan_positions=self.recovery_scan_positions,
        )

        # Execute the following
        result = follower.follow()

        # Translate the result to service response
        response = FollowPersonSrvResponse()
        response.success = result.success
        response.result_type = result.result_type
        response.distance_traveled = result.distance_traveled
        response.following_duration = result.following_duration

        return response

    def _initialize_recovery_positions(self):
        """Initialize the head scan positions for recovery behavior"""
        # Create scan positions from -60 to +60 degrees (left to right)
        scan_angles = [-60, -30, 0, 30, 60, 30, 0, -30]  # Sweep pattern

        self.recovery_scan_positions = []
        for angle in scan_angles:
            # Convert angle to point in base_link frame
            angle_rad = math.radians(angle)
            x = 3.0 * math.cos(angle_rad)  # 3 meters forward distance
            y = 3.0 * math.sin(angle_rad)  # Left/right based on angle
            z = 1.3  # Eye level height

            point = Point(x=x, y=y, z=z)
            self.recovery_scan_positions.append(point)

    def image_callback(
        self, image: Image, depth_image: Image, depth_camera_info: CameraInfo
    ):
        """Store camera data for use by the follower"""
        self.image, self.depth_image, self.depth_camera_info = (
            image,
            depth_image,
            depth_camera_info,
        )


class PersonFollower:
    """Person following implementation using SAM2 and YOLO detection."""

    def __init__(
        self,
        # Default parameters
        start_following_radius: float = 2.0,
        start_following_angle: float = 45.0,
        target_boundary: float = 1.0,
        new_goal_threshold_min: float = 0.25,
        new_goal_threshold_max: float = 2.5,
        stopping_distance: float = 0.75,
        static_speed: float = 0.0015,
        max_speed: float = 0.4,
        max_following_distance: float = 2.5,
        speak: bool = True,
        timeout: float = 0.0,  # 0 means no timeout
        # Clients and publishers from the service
        move_base_client=None,
        point_head_client=None,
        tts_client=None,
        prompt_pub=None,
        track_flag_pub=None,
        condition_frame_flag_pub=None,
        yolo=None,
        buffer=None,
        dynamic_costmap=None,
        dynamic_velocity=None,
        dynamic_recovery=None,
        dynamic_local_costmap=None,
        recovery_scan_positions=None,
    ):
        # Store configuration parameters
        self._start_following_radius = start_following_radius
        self._start_following_angle = start_following_angle
        self._target_boundary = target_boundary
        self._new_goal_threshold_min = new_goal_threshold_min
        self._new_goal_threshold_max = new_goal_threshold_max
        self._stopping_distance = stopping_distance
        self._static_speed = static_speed
        self._max_speed = max_speed
        self._max_following_distance = max_following_distance
        self._speak = speak
        self._timeout = timeout

        # Store clients and publishers
        self._move_base_client = move_base_client
        self._point_head_client = point_head_client
        self._tts_client = tts_client
        self.prompt_pub = prompt_pub
        self.track_flag_pub = track_flag_pub
        self.condition_frame_flag_pub = condition_frame_flag_pub
        self.yolo = yolo
        self._buffer = buffer
        self._dynamic_costmap = dynamic_costmap
        self._dynamic_velocity = dynamic_velocity
        self._dynamic_recovery = dynamic_recovery
        self._dynamic_local_costmap = dynamic_local_costmap
        self.recovery_scan_positions = recovery_scan_positions

        # Initialize tracking state
        self._track_bbox = None
        self._track_id = None
        self._condition_flag_state = True
        self._first_tracking_done = False

        # Subscribe to SAM2 detections during following
        self.detection3d_sub = None

        # Initialise target queue and the newest detection
        self.target_list = []
        self.person_trajectory = PoseArray()
        self.person_trajectory.header.frame_id = "odom"
        self.newest_detection = None

        # Initialise timers
        self.good_detection_timeout_duration = rospy.Duration(5.0)
        self.target_moving_timeout_duration = rospy.Duration(10.0)
        self.last_movement_time = rospy.Time.now()
        self.good_detection_time = rospy.Time.now()
        self.added_new_target_time = rospy.Time.now()
        self.look_at_point_time = rospy.Time.now()
        self.look_down_time = rospy.Time.now()
        self.distance_warning_interval = rospy.Duration(
            5.0
        )  # How often to warn about distance
        self.last_distance_warning_time = rospy.Time.now()
        self.start_time = rospy.Time.now()  # For tracking duration

        # Initialise flags
        self.is_tracking = False
        self.is_navigating = False

        # Recovery behavior variables
        self.is_in_recovery_mode = False
        self.current_scan_index = 0
        self.recovery_start_time = None
        self.scan_position_duration = rospy.Duration(1.5)
        self.last_scan_position_time = rospy.Time.now()
        self.recovery_timeout = rospy.Duration(30.0)

        # Configure navigation parameters
        self._configure_navigation()

        # Track the distance traveled
        self.distance_traveled = 0.0
        self.last_position = None

    def _configure_navigation(self):
        """Configure navigation parameters for person following"""
        # Set costmap size to 4x4 meters
        config = Config()
        config.ints.append(IntParameter(name="width", value=4))
        config.ints.append(IntParameter(name="height", value=4))
        self._dynamic_costmap(config)

        # Set maximum velocity
        config = Config()
        config.doubles.append(DoubleParameter(name="max_vel_x", value=self._max_speed))
        self._dynamic_velocity(config)

        # Disable recovery behaviors and rotation clearing
        config = Config()
        config.bools.append(BoolParameter(name="recovery_behavior_enabled", value=0))
        config.bools.append(BoolParameter(name="clearing_rotation_allowed", value=0))
        self._dynamic_recovery(config)

        # Set local costmap inflation radius
        config = Config()
        config.bools.append(BoolParameter(name="enabled", value=1))
        config.doubles.append(DoubleParameter(name="inflation_radius", value=0.2))
        self._dynamic_local_costmap(config)

        # Clear existing costmaps
        rospy.ServiceProxy("/move_base/clear_costmaps", Empty)()

        rospy.loginfo("Navigation parameters configured for person following")

    def _tts(self, text: str, wait: bool) -> None:
        """Speak text if speech is enabled"""
        if not self._speak:
            return

        if self._tts_client and hasattr(self._tts_client, "wait_for_server"):
            tts_goal = TtsGoal()
            tts_goal.rawtext.text = text
            tts_goal.rawtext.lang_id = "en_GB"
            if wait:
                self._tts_client.send_goal_and_wait(tts_goal)
            else:
                self._tts_client.send_goal(tts_goal)

    def _look_at_point(self, target_point: Point, target_frame: str = "map"):
        """Point the head at a specific point"""
        goal = PointHeadGoal()
        # Use timestamp from TF for better synchronization
        goal.target.header.stamp = rospy.Time(0)  # Use latest transform available
        goal.target.header.frame_id = target_frame
        goal.target.point = target_point

        goal.pointing_frame = "xtion_rgb_optical_frame"
        goal.pointing_axis.x = 0.0
        goal.pointing_axis.y = 0.0
        goal.pointing_axis.z = 1.0

        goal.min_duration = rospy.Duration(0.1)
        goal.max_velocity = 0.33

        self._point_head_client.send_goal(goal)

    def _look_centre_point(self):
        """Look straight ahead at eye level"""
        point = Point(x=3.0, y=0.0, z=1.3)
        self._look_at_point(point, target_frame="base_link")

    def _move_head(self, target_point: Point, target_frame: str = "map"):
        """Move head to look at target with rate limiting"""
        current_time = rospy.Time.now()
        if current_time - self.look_at_point_time >= rospy.Duration(0.35):
            self._look_at_point(target_point, target_frame=target_frame)
            self.look_at_point_time = current_time

    def _tf_pose(self, pose: PoseStamped, target_frame: str):
        """Transform a pose to the target frame"""
        trans = self._buffer.lookup_transform(
            target_frame, pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0)
        )
        return do_transform_pose(pose, trans)

    def _robot_pose_in_odom(self) -> PoseStamped:
        """Get the current robot pose in odom frame"""
        try:
            # Look up the transform from base_link to odom
            transform = self._buffer.lookup_transform(
                "odom",  # target frame
                "base_link",  # source frame
                rospy.Time(0),  # get latest available transform
                rospy.Duration(1.0),  # timeout
            )

            # Create a PoseStamped message for the robot's current pose
            robot_pose = PoseStamped()
            robot_pose.header.frame_id = "odom"
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

            # Update distance traveled
            if self.last_position is not None:
                dx = robot_pose.pose.position.x - self.last_position.x
                dy = robot_pose.pose.position.y - self.last_position.y
                distance = math.sqrt(dx * dx + dy * dy)
                self.distance_traveled += distance

            # Store current position for next calculation
            self.last_position = robot_pose.pose.position

            return robot_pose

        except (
            tf.LookupException,
            tf.ExtrapolationException,
            tf.ConnectivityException,
        ) as e:
            rospy.logerr(f"Failed to get robot pose in odom frame: {e}")
            raise

    def _euclidian_distance(
        self, p1: Union[Pose, PoseStamped], p2: Union[Pose, PoseStamped]
    ) -> float:
        """Calculate Euclidean distance between two poses"""
        # Extract Pose from PoseStamped if necessary
        pose1 = p1.pose if isinstance(p1, PoseStamped) else p1
        pose2 = p2.pose if isinstance(p2, PoseStamped) else p2

        return np.linalg.norm(
            np.array([pose1.position.x, pose1.position.y])
            - np.array([pose2.position.x, pose2.position.y])
        ).astype(float)

    def _compute_face_quat(self, p1: Pose, p2: Pose) -> Quaternion:
        """Compute quaternion to face from p1 to p2"""
        dx: float = p2.position.x - p1.position.x
        dy: float = p2.position.y - p1.position.y
        theta_deg = np.degrees(atan2(dy, dx))
        x, y, z, w = R.from_euler("z", theta_deg, degrees=True).as_quat()
        return Quaternion(x, y, z, w)

    def _cancel_goal(self) -> None:
        """Cancel the current navigation goal"""
        if self._move_base_client.get_state() in [
            GoalStatus.PENDING,
            GoalStatus.ACTIVE,
        ]:
            self._move_base_client.cancel_goal()
            self._move_base_client.wait_for_result()
            self.is_navigating = False
            self.last_movement_time = rospy.Time.now()
            rospy.loginfo("Goal cancelled.")

    def _move_base(self, pose: PoseStamped, wait=False) -> MoveBaseGoal:
        """Send a goal to move_base"""
        goal: MoveBaseGoal = MoveBaseGoal()
        goal.target_pose = pose
        self._move_base_client.send_goal(goal)
        if wait:
            self._move_base_client.wait_for_result()
        return goal

    def detection3d_callback(self, msg: Detection3DArray):
        """Process SAM2 detections to track the target person"""
        for detection in msg.detections:
            if int(detection.name) == self._track_id:
                # Get the bounding box dimensions from xywh
                if hasattr(detection, "xywh") and len(detection.xywh) == 4:
                    xywh = detection.xywh

                    # Calculate bounding box properties
                    box_width = xywh[2]
                    box_height = xywh[3]
                    box_area = box_width * box_height

                    # Get image dimensions (assuming standard camera resolution if not available)
                    image_width = 640  # Default width if not available
                    image_height = 480  # Default height if not available

                    # Calculate center position of the box
                    center_x = xywh[0] + box_width / 2
                    center_y = xywh[1] + box_height / 2

                    # Calculate normalized distance from center of image
                    image_center_x = image_width / 2
                    image_center_y = image_height / 2
                    normalized_distance = math.sqrt(
                        ((center_x - image_center_x) / image_width) ** 2
                        + ((center_y - image_center_y) / image_height) ** 2
                    )

                    # Calculate normalized area
                    normalized_area = box_area / (image_width * image_height)

                    # Define thresholds for quality check
                    max_distance_threshold = 0.9
                    min_area_threshold = 0.01

                    # Calculate additional quality metrics
                    aspect_ratio = box_width / max(box_height, 1)
                    is_reasonable_aspect = True

                    # Check quality criteria
                    is_good_quality = (
                        normalized_distance < max_distance_threshold
                        and normalized_area > min_area_threshold
                        and is_reasonable_aspect
                        and detection.confidence > 0.5
                    )

                    if (
                        detection is not None
                        and is_reasonable_aspect
                        and detection.confidence > 0.5
                    ):
                        self.newest_detection = detection

                    if self.newest_detection is not None:
                        self._move_head(self.newest_detection.point, target_frame="map")

                    # Set conditional frame flag based on quality
                    if is_good_quality:
                        # Good quality detection - allow conditional frames
                        self.condition_frame_flag_pub.publish(Bool(data=True))
                        self._condition_flag_state = True
                        self.good_detection_time = rospy.Time.now()

                        # Transform into the odom frame for trajectory
                        map_pose = PoseStamped()
                        map_pose.header.frame_id = "map"
                        map_pose.header.stamp = rospy.Time.now()
                        map_pose.pose.position = detection.point
                        map_pose.pose.orientation.w = 1.0  # Identity orientation

                        try:
                            odom_pose = self._buffer.transform(
                                map_pose, "odom", rospy.Duration(0.50)
                            )
                        except (tf.LookupException, tf.ExtrapolationException) as e:
                            rospy.logwarn(f"TF transform failed: {e}")
                            return

                        # Add to trajectory if appropriate
                        if len(self.target_list) == 0:
                            # Add position to trajectory
                            self.person_trajectory.poses.append(odom_pose.pose)
                            self.target_list.append(odom_pose.pose)
                            self.added_new_target_time = rospy.Time.now()
                        else:
                            prev_pose = self.target_list[-1]
                            dist_to_prev = self._euclidian_distance(
                                prev_pose, odom_pose.pose
                            )
                            # Only add if distance exceeds threshold
                            if (
                                self._new_goal_threshold_min
                                < dist_to_prev
                                < self._new_goal_threshold_max
                            ):
                                self.person_trajectory.poses.append(odom_pose.pose)
                                self.target_list.append(odom_pose.pose)
                                self.added_new_target_time = rospy.Time.now()
                    else:
                        # Poor quality detection - disable conditional frames
                        self.condition_frame_flag_pub.publish(Bool(data=False))
                        self._condition_flag_state = False
                break

    def begin_tracking(self) -> bool:
        """Find and start tracking the closest person"""
        # Initialize head position
        self._look_centre_point()
        rospy.sleep(1.0)

        # Wait for camera images
        timeout = rospy.Time.now() + rospy.Duration(5.0)
        while rospy.Time.now() < timeout:
            # Check if images are available
            if (
                hasattr(self, "image")
                and hasattr(self, "depth_image")
                and hasattr(self, "depth_camera_info")
            ):
                if self.image and self.depth_image and self.depth_camera_info:
                    break
            rospy.sleep(0.1)

        if not (self.image and self.depth_image and self.depth_camera_info):
            rospy.logerr("Failed to get camera images within timeout")
            return False

        # Detect people using YOLO
        req = YoloDetection3DRequest(
            image_raw=self.image,
            depth_image=self.depth_image,
            depth_camera_info=self.depth_camera_info,
            model="yolo11n-pose.pt",
            confidence=0.5,
            target_frame="map",
        )

        try:
            response = self.yolo(req)
            detections = response.detected_objects
        except Exception as e:
            rospy.logerr(f"YOLO detection failed: {e}")
            return False

        # Extract people information
        detected_people = {
            "xywh": [],
            "point": [],
        }
        for detection in detections:
            detected_people["point"].append(detection.point)
            detected_people["xywh"].append(detection.xywh)

        if not detected_people["point"]:
            rospy.logwarn("No people detected")
            return False

        # Find the nearest person
        robot_frame = "base_link"
        map_frame = "map"

        # Find robot's position in map
        try:
            transform = self._buffer.lookup_transform(
                map_frame, robot_frame, rospy.Time(0), rospy.Duration(1.0)
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
        except tf.LookupException as e:
            rospy.logerr(f"TF lookup failed: {e}")
            return False

        min_dist = float("inf")
        nearest_index = -1

        for i, point in enumerate(detected_people["point"]):
            dx = point.x - robot_x
            dy = point.y - robot_y
            dist = math.hypot(dx, dy)

            if dist < min_dist:
                min_dist = dist
                nearest_index = i

        if nearest_index != -1:
            nearest_person_point = detected_people["point"][nearest_index]
            nearest_person_bbox = detected_people["xywh"][nearest_index]
            rospy.loginfo(f"Nearest person at distance {min_dist:.2f}m")
        else:
            rospy.logwarn("Failed to find nearest person")
            return False

        # Start tracking
        self._track_bbox = nearest_person_bbox
        self._track_id = nearest_index

        # Prepare and send SAM2 tracking command
        bbox_list = []
        for i, bbox in enumerate(detected_people["xywh"]):
            bbox_msg = BboxWithFlag()
            bbox_msg.obj_id = i
            bbox_msg.reset = False  # not to reset here, will reset together
            bbox_msg.clear_old_points = True
            bbox_msg.xywh = bbox
            bbox_list.append(bbox_msg)
            rospy.loginfo(f"Prepared BBox for ID {i}")

        # Initialize target queue and tracking state
        self.target_list = []
        self.person_trajectory = PoseArray()
        self.person_trajectory.header.frame_id = "odom"
        self.last_movement_time = rospy.Time.now()
        self.good_detection_time = rospy.Time.now()
        self.added_new_target_time = rospy.Time.now()
        self.newest_detection = None

        # Subscribe to SAM2 detections
        self.detection3d_sub = rospy.Subscriber(
            "/sam2/detections_3d",
            Detection3DArray,
            self.detection3d_callback,
            queue_size=1,
        )

        # Send tracking commands to SAM2
        prompt_msg = PromptArrays()
        prompt_msg.bbox_array = bbox_list
        prompt_msg.point_array = []
        prompt_msg.reset = True  # full reset
        self.prompt_pub.publish(prompt_msg)
        rospy.loginfo(f"Published PromptArrays with {len(bbox_list)} BBoxes")
        rospy.sleep(0.5)

        self.track_flag_pub.publish(Bool(data=True))
        rospy.loginfo(f"Published SAM2 tracking command")
        rospy.sleep(0.1)

        self.condition_frame_flag_pub.publish(Bool(data=True))
        rospy.loginfo(f"Tracking person with id {self._track_id}")

        return True

    def _start_recovery_behavior(self):
        """Start scanning for lost target"""
        if not self.is_in_recovery_mode:
            rospy.loginfo("Starting recovery behavior - scanning for lost target")
            self._tts("Let me look for you.", wait=False)

            self.is_in_recovery_mode = True
            self.current_scan_index = 0
            self.recovery_start_time = rospy.Time.now()
            self.last_scan_position_time = rospy.Time.now()

            # Start with first scan position
            if self.recovery_scan_positions:
                self._look_at_point(
                    self.recovery_scan_positions[0], target_frame="base_link"
                )

    def _update_recovery_behavior(self):
        """Update scanning during recovery mode"""
        if not self.is_in_recovery_mode:
            return False

        current_time = rospy.Time.now()

        # Check if recovery has timed out
        if current_time - self.recovery_start_time > self.recovery_timeout:
            rospy.logwarn("Recovery behavior timed out")
            self._stop_recovery_behavior()
            return False

        # Check if it's time to move to next scan position
        if current_time - self.last_scan_position_time > self.scan_position_duration:
            self.current_scan_index += 1

            # If we've completed all scan positions, start over
            if self.current_scan_index >= len(self.recovery_scan_positions):
                self.current_scan_index = 0
                rospy.loginfo("Completed one full scan cycle, starting over")

            # Move to next scan position
            scan_point = self.recovery_scan_positions[self.current_scan_index]
            self._look_at_point(scan_point, target_frame="base_link")
            self.last_scan_position_time = current_time

        return True

    def _stop_recovery_behavior(self):
        """Stop recovery scanning"""
        if self.is_in_recovery_mode:
            rospy.loginfo("Stopping recovery behavior")
            self.is_in_recovery_mode = False
            self.current_scan_index = 0
            self.recovery_start_time = None

            # Return head to center position
            self._look_centre_point()

            # Reset the time of recovery to avoid stop detection
            self.last_movement_time = rospy.Time.now()

    def _check_target_recovery(self):
        """Check if target was found during recovery"""
        current_time = rospy.Time.now()

        # If we have a recent good detection, target is recovered
        if current_time - self.good_detection_time < rospy.Duration(2.0):
            if self.is_in_recovery_mode:
                rospy.loginfo("Target recovered during scanning!")
                self._tts("Found you! Continuing to follow.", wait=False)
                self._stop_recovery_behavior()
                return True

        return False

    def cleanup(self):
        """Clean up resources before ending service"""
        # Cancel any active navigation goal
        self._cancel_goal()

        # Unsubscribe from detection topics
        if self.detection3d_sub:
            self.detection3d_sub.unregister()

        # Stop SAM2 tracking
        self.track_flag_pub.publish(Bool(data=False))

        # Reset head position
        self._look_centre_point()

        rospy.loginfo("Person following resources cleaned up")

    def follow(self) -> dict:
        """
        Main method to execute person following

        Returns:
            dict: Result of the following operation with success flag, reason and stats
        """
        result = {
            "success": False,
            "result_type": "UNKNOWN",
            "distance_traveled": 0.0,
            "following_duration": 0.0,
        }

        # Initialize state
        rate = rospy.Rate(10)  # 10 Hz
        self.last_movement_time = rospy.Time.now()
        self.good_detection_time = rospy.Time.now()
        self.start_time = rospy.Time.now()
        previous_target = None
        just_started = True

        try:
            # Main following loop
            while not rospy.is_shutdown():
                current_time = rospy.Time.now()

                # Check timeout if specified
                if (
                    self._timeout > 0
                    and (current_time - self.start_time).to_sec() > self._timeout
                ):
                    rospy.loginfo(
                        f"Person following timed out after {self._timeout} seconds"
                    )
                    self._tts(
                        "I've been following for the maximum time. Stopping now.",
                        wait=False,
                    )
                    result["result_type"] = "TIMEOUT"
                    result["success"] = True
                    break

                # Check if target has been recovered during scanning
                if self._check_target_recovery():
                    # Target was recovered, continue normal operation
                    pass

                # Check for bad detection or start recovery behavior
                if (
                    self.good_detection_time + self.target_moving_timeout_duration
                    < current_time
                ):
                    if not self.is_in_recovery_mode:
                        rospy.loginfo("Tracking stopped for no good detection")
                        self._tts("I cannot find you anymore.", wait=True)
                        self._start_recovery_behavior()
                    else:
                        # Continue recovery behavior
                        if not self._update_recovery_behavior():
                            # Recovery timed out or failed
                            self._tts(
                                "I give up looking for you. Please come back.",
                                wait=True,
                            )
                            result["result_type"] = "LOST_PERSON"
                            break
                else:
                    # Good detection - stop recovery if active
                    if self.is_in_recovery_mode:
                        self._stop_recovery_behavior()

                # Skip normal navigation logic if in recovery mode
                if self.is_in_recovery_mode:
                    rate.sleep()
                    continue

                # Check if we've stopped moving
                if not self.is_navigating and not just_started:
                    if (
                        self.last_movement_time + self.target_moving_timeout_duration
                        < current_time
                    ):
                        rospy.loginfo("Tracking stopped for no movement")
                        self._tts("Have we arrived? I will stop following.", wait=True)
                        result["result_type"] = "ARRIVED"
                        result["success"] = True
                        break

                # Start tracking if not already tracking
                if not self.is_tracking:
                    self.is_tracking = self.begin_tracking()
                    if self.is_tracking:
                        self._tts("I will start to follow you.", wait=True)
                    rate.sleep()
                    continue

                # Check navigation state - if we reached a goal, start the timer
                if self.is_navigating:
                    nav_state = self._move_base_client.get_state()
                    if nav_state not in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
                        self.is_navigating = False
                        self.last_movement_time = rospy.Time.now()
                        rospy.loginfo("Goal reached")

                # Navigate to the next target
                if len(self.target_list) == 0:
                    rospy.logwarn("No target list available")
                    rate.sleep()
                    continue

                last_pose_in_list = self.target_list[-1]
                target_pose = None

                # Find appropriate target pose
                for i in reversed(range(len(self.target_list))):
                    if self._target_boundary <= self._euclidian_distance(
                        self.target_list[i], last_pose_in_list
                    ):
                        target_pose = self.target_list[i]
                        break

                if target_pose is None:
                    rospy.logwarn("No suitable target pose available")
                    rate.sleep()
                    continue

                if (
                    previous_target
                    and self._euclidian_distance(target_pose, previous_target)
                    < self._new_goal_threshold_min
                ):
                    rospy.logwarn("Target pose is too similar to previous target pose")
                    rate.sleep()
                    continue

                # Only send new navigation goal if needed
                if not self.is_navigating:
                    try:
                        robot_pose = self._robot_pose_in_odom()
                        distance_to_target = self._euclidian_distance(
                            robot_pose.pose, target_pose
                        )

                        # Check if target is too far away - ask person to wait
                        if distance_to_target > self._max_following_distance:
                            # Only send warning if enough time has passed since last warning
                            if (
                                current_time - self.last_distance_warning_time
                                > self.distance_warning_interval
                            ):
                                rospy.loginfo(
                                    f"Target too far ({distance_to_target:.2f}m > {self._max_following_distance}m)"
                                )
                                self._tts(
                                    "Please wait for me. You are too far away.",
                                    wait=False,
                                )
                                self.last_distance_warning_time = current_time

                        # If robot is already close enough to target, don't navigate
                        if distance_to_target < self._stopping_distance:
                            rospy.logdebug(
                                f"Already close to target ({distance_to_target:.2f}m < {self._stopping_distance}m)"
                            )
                            rate.sleep()
                            continue

                    except Exception as e:
                        rospy.logwarn(f"Failed to get robot pose: {e}")

                    # Set orientation to face the final pose
                    goal_orientation = self._compute_face_quat(
                        target_pose, last_pose_in_list
                    )
                    pose_with_orientation = Pose(
                        position=target_pose.position,
                        orientation=goal_orientation,
                    )

                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = "odom"
                    pose_stamped.header.stamp = rospy.Time.now()
                    pose_stamped.pose = pose_with_orientation

                    goal_pose = self._tf_pose(pose_stamped, "map")
                    rospy.loginfo("Setting navigation goal to intermediate point")
                    self._move_base(goal_pose)
                    self.is_navigating = True
                    just_started = False
                    previous_target = target_pose  # Update previous target

                rate.sleep()

            # End of while loop - prepare final result
            if result["result_type"] == "UNKNOWN":
                result["result_type"] = (
                    "CANCELLED"  # Default if not otherwise specified
                )

            # Calculate statistics
            result["distance_traveled"] = self.distance_traveled
            result["following_duration"] = (rospy.Time.now() - self.start_time).to_sec()

            return result

        except Exception as e:
            rospy.logerr(f"Error during person following: {e}")
            result["result_type"] = "ERROR"
            return result

        finally:
            # Always clean up
            self.cleanup()


if __name__ == "__main__":
    service = PersonFollowerService()
    rospy.loginfo("Person following service started")
    rospy.spin()
