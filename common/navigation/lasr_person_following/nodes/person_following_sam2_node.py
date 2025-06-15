#!/usr/bin/env python3
import rospy
import math

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

import rosservice
import tf2_ros as tf
import tf2_geometry_msgs  # type: ignore
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from nav_msgs.srv import GetPlan
from play_motion_msgs.msg import PlayMotionAction
from play_motion_msgs.msg import PlayMotionGoal
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from lasr_vision_msgs.msg import Sam2PromptArrays as PromptArrays, Sam2BboxWithFlag as BboxWithFlag, Detection3DArray, \
    Detection3D
from lasr_vision_msgs.srv import YoloDetection3D, YoloDetection3DRequest

from math import atan2
import numpy as np
from scipy.spatial.transform import Rotation as R

import actionlib
from control_msgs.msg import PointHeadAction, PointHeadGoal

from lasr_person_following.msg import (
    FollowAction,
    FollowGoal,
    FollowResult,
    FollowFeedback,
)

MAX_VISION_DIST: float = 5.0


class PersonFollower:
    """ROS node for person following using SAM2 and YOLO detection."""

    def __init__(
            self,
            start_following_radius: float = 2.0,  # Distance threshold to start following (meters)
            start_following_angle: float = 45.0,  # Angle threshold to start following (degrees)
            target_boundary: float = 1.0,
            new_goal_threshold_min: float = 0.25,  # Minimum distance to trigger new navigation goal (meters)
            new_goal_threshold_max: float = 2.5,  # Max distance to trigger new navigation goal (meters)
            stopping_distance: float = 0.75,  # Distance to maintain from person (meters)
            static_speed: float = 0.0015,  # Speed when person is stationary (m/s)
            max_speed: float = 0.4,  # Maximum robot velocity (m/s)
            max_following_distance: float = 2.5,  # Maximum distance to follow before asking person to wait (meters)
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

        # Initialize tracking state
        self._track_bbox = None
        self._track_id = None

        # Initialize ROS node
        rospy.init_node("person_following_sam2")

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
        self.prompt_pub = rospy.Publisher("/sam2/prompt_arrays", PromptArrays, queue_size=1)
        self.track_flag_pub = rospy.Publisher("/sam2/track_flag", Bool, queue_size=1)
        self.detection3d_sub = rospy.Subscriber(
            "/sam2/detections_3d", Detection3DArray, self.detection3d_callback, queue_size=1
        )

        # Setup SAM2 conditioning frame control
        self.condition_frame_flag_pub = rospy.Publisher("/sam2/add_conditioning_frame_flag", Bool, queue_size=1)
        self._condition_flag_state = True
        self._first_tracking_done = False

        # Setup visualization and YOLO service
        self.trajectory_marker_pub = rospy.Publisher(
            "/person_trajectory_markers", MarkerArray, queue_size=1
        )
        self.yolo = rospy.ServiceProxy("/yolo/detect3d", YoloDetection3D)
        self.yolo.wait_for_service()

        # Setup navigation client
        self._move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        if not self._move_base_client.wait_for_server(rospy.Duration.from_sec(10.0)):
            rospy.logwarn("Move base client not available")

        # Setup TF buffer and listener
        self._buffer = tf.Buffer(cache_time=rospy.Duration.from_sec(10.0))
        self._listener = tf.TransformListener(self._buffer)

        # Initialise target queue and the newest detection
        self.target_list: List[Tuple] = []
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

        # Initialise flags
        self.is_tracking = False
        self.is_navigating = False

        # Setup head pointing client
        self._point_head_client = actionlib.SimpleActionClient(
            "/head_controller/point_head_action", PointHeadAction
        )
        if not self._point_head_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logwarn("Head pointing action server not available.")

        # Setup scanning parameters
        self._scan_interval = rospy.Duration(3.0)
        self._last_scan_time = rospy.Time.now()

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
        rospy.wait_for_service("/move_base/local_costmap/inflation_layer/set_parameters")
        rospy.wait_for_service("/move_base/clear_costmaps")

        # Configure navigation parameters
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

        rospy.sleep(1)
        rospy.loginfo("Dynamic parameters updated.")

        # Setup TTS client
        self._tts_client = actionlib.SimpleActionClient("tts", TtsAction)
        self._tts_client_available = self._tts_client.wait_for_server(
            rospy.Duration.from_sec(10.0)
        )
        if not self._tts_client_available:
            rospy.logwarn("TTS client not available")

        # Add recovery behavior variables
        self.is_in_recovery_mode = False
        self.recovery_scan_positions = []  # List of head positions for scanning
        self.current_scan_index = 0
        self.recovery_start_time = None
        self.scan_position_duration = rospy.Duration(1.5)  # Time to hold each scan position
        self.last_scan_position_time = rospy.Time.now()
        self.recovery_timeout = rospy.Duration(30.0)  # Maximum time to spend in recovery mode

        # Initialize recovery scan positions (60 degrees left to 60 degrees right)
        self._initialize_recovery_positions()

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

    def _tts(self, text: str, wait: bool) -> None:
        if self._tts_client_available:
            tts_goal: TtsGoal = TtsGoal()
            tts_goal.rawtext.text = text
            tts_goal.rawtext.lang_id = "en_GB"
            if wait:
                self._tts_client.send_goal_and_wait(tts_goal)
            else:
                self._tts_client.send_goal(tts_goal)

    def _look_at_point(self, target_point: Point, target_frame: str = "map"):
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
        point = Point(x=3.0, y=0.0, z=1.3)
        self._look_at_point(point, target_frame="base_link")

    def _look_down_centre_point(self):
        point = Point(x=3.0, y=0.0, z=0.5)
        self._look_at_point(point, target_frame="base_link")

    def _look_down_point(self, target_point: Point = None, target_frame: str = "map"):
        """
        Look down towards the target direction or center if no target provided.

        Args:
            target_point: Target point to look towards (optional)
            target_frame: Frame of the target point
        """
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
                    norm = math.sqrt(x_dir ** 2 + y_dir ** 2)
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

    def _move_head(self, target_point: Point, target_frame: str = "map"):
        current_time = rospy.Time.now()

        # if current_time - self.look_down_time >= rospy.Duration(3.1):
        #     self._look_down_point(target_point, target_frame=target_frame)
        #     self.look_down_time = current_time
        #     return
        #
        # look_down_protection_time = rospy.Duration(1.5)
        # if current_time - self.look_down_time < look_down_protection_time:
        #     return

        if current_time - self.look_at_point_time >= rospy.Duration(0.35):
            self._look_at_point(target_point, target_frame=target_frame)
            self.look_at_point_time = current_time

    def image_callback(self, image: Image, depth_image: Image, depth_camera_info: CameraInfo):
        self.image, self.depth_image, self.depth_camera_info = image, depth_image, depth_camera_info

    def _tf_pose(self, pose: PoseStamped, target_frame: str):
        trans = self._buffer.lookup_transform(
            target_frame, pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0)
        )
        return do_transform_pose(pose, trans)

    def _robot_pose_in_odom(self) -> PoseStamped:
        """
        Get the current robot pose in the odom coordinate frame.

        Returns:
            PoseStamped: Current robot pose in odom frame

        Raises:
            tf.LookupException: If the transform cannot be found
            tf.ExtrapolationException: If the transform is too old
        """
        try:
            # Look up the transform from base_link to odom
            transform = self._buffer.lookup_transform(
                "odom",  # target frame
                "base_link",  # source frame
                rospy.Time(0),  # get latest available transform
                rospy.Duration(1.0)  # timeout
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

            return robot_pose

        except (tf.LookupException, tf.ExtrapolationException, tf.ConnectivityException) as e:
            rospy.logerr(f"Failed to get robot pose in odom frame: {e}")
            raise

    def _euclidian_distance(self, p1: Union[Pose, PoseStamped], p2: Union[Pose, PoseStamped]) -> float:
        """
        Calculate Euclidean distance between two poses.

        Args:
            p1: First pose (Pose or PoseStamped)
            p2: Second pose (Pose or PoseStamped)

        Returns:
            float: Euclidean distance in meters
        """
        # Extract Pose from PoseStamped if necessary
        pose1 = p1.pose if isinstance(p1, PoseStamped) else p1
        pose2 = p2.pose if isinstance(p2, PoseStamped) else p2

        return np.linalg.norm(
            np.array([pose1.position.x, pose1.position.y])
            - np.array([pose2.position.x, pose2.position.y])
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
            self._move_base_client.wait_for_result()
            self.is_navigating = False
            self.last_movement_time = rospy.Time.now()
            rospy.loginfo("Goal cancelled.")

    def _move_base(self, pose: PoseStamped, wait=False) -> MoveBaseGoal:
        goal: MoveBaseGoal = MoveBaseGoal()
        goal.target_pose = pose
        self._move_base_client.send_goal(goal)
        if wait:
            self._move_base_client.wait_for_result()
        return goal

    def detection3d_callback(self, msg: Detection3DArray):
        for detection in msg.detections:
            if int(detection.name) == self._track_id:
                # Get the bounding box dimensions from xywh
                # xywh format: [x_min, y_min, width, height]
                if hasattr(detection, 'xywh') and len(detection.xywh) == 4:
                    xywh = detection.xywh

                    # Calculate bounding box properties
                    box_width = xywh[2]
                    box_height = xywh[3]
                    box_area = box_width * box_height

                    # Get image dimensions (assuming standard camera resolution if not available)
                    image_width = 640  # Default width if not available
                    image_height = 480  # Default height if not available
                    if hasattr(self, 'depth_camera_info') and self.depth_camera_info is not None:
                        image_width = self.depth_camera_info.width
                        image_height = self.depth_camera_info.height

                    # Calculate center position of the box
                    center_x = xywh[0] + box_width / 2
                    center_y = xywh[1] + box_height / 2

                    # Calculate normalized distance from center of image
                    image_center_x = image_width / 2
                    image_center_y = image_height / 2
                    normalized_distance = math.sqrt(
                        ((center_x - image_center_x) / image_width) ** 2 +
                        ((center_y - image_center_y) / image_height) ** 2
                    )

                    # Calculate normalized area
                    normalized_area = box_area / (image_width * image_height)

                    # Define thresholds for quality check
                    # Too far from center or too small detection is considered poor quality
                    max_distance_threshold = 0.9  # Maximum allowed normalized distance from center (reduced from 1.0)
                    min_area_threshold = 0.01  # Minimum normalized area required

                    # Calculate additional quality metrics (optional)
                    aspect_ratio = box_width / max(box_height, 1)  # Avoid division by zero
                    is_reasonable_aspect = True  # 0.2 < aspect_ratio < 5.0  # Person should have reasonable aspect ratio

                    # Check quality criteria
                    is_good_quality = (normalized_distance < max_distance_threshold and
                                       normalized_area > min_area_threshold and
                                       is_reasonable_aspect and detection.confidence > 0.5)

                    if detection is not None and is_reasonable_aspect and detection.confidence > 0.5:
                        self.newest_detection = detection

                    if self.newest_detection is not None:
                        self._move_head(self.newest_detection.point, target_frame="map")

                    # Set conditional frame flag based on quality
                    if is_good_quality:
                        # Good quality detection - allow conditional frames
                        # if not self._condition_flag_state:
                        self.condition_frame_flag_pub.publish(Bool(data=True))
                        self._condition_flag_state = True
                        # Log quality metrics at debug level
                        rospy.logdebug(f"Good detection - Distance: {normalized_distance:.3f}, "
                                       f"Area: {normalized_area:.5f}, Aspect: {aspect_ratio:.2f}")
                        self.good_detection_time = rospy.Time.now()

                        # transpose into the map point
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

                        # if target list is empty or the distance make sense
                        if len(self.target_list) == 0:
                            # Add position to trajectory
                            self.person_trajectory.poses.append(odom_pose.pose)
                            self.target_list.append(odom_pose.pose)
                            self.added_new_target_time = rospy.Time.now()
                        else:
                            prev_pose = self.target_list[-1]
                            dist_to_prev = self._euclidian_distance(prev_pose, odom_pose.pose)
                            # Only add if distance exceeds threshold
                            if self._new_goal_threshold_min < dist_to_prev < self._new_goal_threshold_max:
                                self.person_trajectory.poses.append(odom_pose.pose)
                                self.target_list.append(odom_pose.pose)
                                self.added_new_target_time = rospy.Time.now()
                    else:
                        # Poor quality detection - disable conditional frames
                        # if self._condition_flag_state:
                        self.condition_frame_flag_pub.publish(Bool(data=False))
                        self._condition_flag_state = False
                        rospy.loginfo(f"Detection quality poor - disabling conditional frames")

                        # Log the specific reason for poor quality
                        quality_issues = []
                        if normalized_distance >= max_distance_threshold:
                            quality_issues.append(f"off-center ({normalized_distance:.3f} >= {max_distance_threshold})")
                        if normalized_area <= min_area_threshold:
                            quality_issues.append(f"too small ({normalized_area:.5f} <= {min_area_threshold})")
                        if not is_reasonable_aspect:
                            quality_issues.append(f"odd shape (aspect ratio: {aspect_ratio:.2f})")

                        if quality_issues:
                            rospy.loginfo(f"Quality issues: {', '.join(quality_issues)}")
                break

    def begin_tracking(self) -> bool:
        """
        Chooses the closest person as the target
        """
        # Initialize head position
        self._look_centre_point()
        rospy.sleep(1.0)

        while True:
            if self.image and self.depth_image and self.depth_camera_info:
                break

        req = YoloDetection3DRequest(
            image_raw=self.image,
            depth_image=self.depth_image,
            depth_camera_info=self.depth_camera_info,
            model="yolo11n-pose.pt",
            confidence=0.5,
            target_frame="map",
        )
        response = self.yolo(req)
        print(response)
        detections: Detection3D = response.detected_objects

        detected_people = {
            "xywh": [],
            "point": [],
        }
        for detection in detections:
            detected_people["point"].append(detection.point)
            detected_people["xywh"].append(detection.xywh)

        # find the nearest person:
        robot_frame = "base_link"
        map_frame = "map"

        # find robot's position in map
        try:
            transform = self._buffer.lookup_transform(
                map_frame,
                robot_frame,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
        except tf.LookupException as e:
            rospy.logerr("TF lookup failed: %s", str(e))
            return

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
            return False

        self._track_bbox = nearest_person_bbox
        self._track_id = nearest_index

        # restart sam2 and prompt it
        bbox_list = []
        for i, bbox in enumerate(detected_people["xywh"]):
            bbox_msg = BboxWithFlag()
            bbox_msg.obj_id = i
            bbox_msg.reset = False  # not to reset here, will reset together
            bbox_msg.clear_old_points = True
            bbox_msg.xywh = bbox
            bbox_list.append(bbox_msg)
            rospy.loginfo(f"Prepared BBox for ID {i}")

        # Initialize target queue and the newest target
        self.target_list: List[Tuple] = []
        self.person_trajectory = PoseArray()
        self.person_trajectory.header.frame_id = "odom"
        self.last_movement_time = rospy.Time.now()
        self.good_detection_time = rospy.Time.now()
        self.added_new_target_time = rospy.Time.now()
        self.newest_detection = None

        prompt_msg = PromptArrays()
        prompt_msg.bbox_array = bbox_list
        prompt_msg.point_array = []
        prompt_msg.reset = True  # full reset
        self.prompt_pub.publish(prompt_msg)
        rospy.loginfo(f"Published PromptArrays with {len(bbox_list)} BBoxes.")
        rospy.sleep(0.5)
        self.track_flag_pub.publish(Bool(data=True))
        rospy.loginfo(f"Published SAM2 traking commmand.")
        rospy.sleep(0.1)
        rospy.loginfo(f"Tracking person discovered with id {self._track_id}")
        self.condition_frame_flag_pub.publish(Bool(data=True))
        return True

    def _start_recovery_behavior(self):
        """Start the recovery scanning behavior when target is lost"""
        if not self.is_in_recovery_mode:
            rospy.loginfo("Starting recovery behavior - scanning for lost target")
            self._tts("Let me look for you.", wait=False)

            self.is_in_recovery_mode = True
            self.current_scan_index = 0
            self.recovery_start_time = rospy.Time.now()
            self.last_scan_position_time = rospy.Time.now()

            # Start with first scan position
            if self.recovery_scan_positions:
                self._look_at_point(self.recovery_scan_positions[0], target_frame="base_link")

    def _update_recovery_behavior(self):
        """Update the recovery scanning behavior"""
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

            rospy.logdebug(
                f"Scanning position {self.current_scan_index}: looking at angle {self.current_scan_index * 30 - 60} degrees")

        return True

    def _stop_recovery_behavior(self):
        """Stop the recovery behavior and return to normal tracking"""
        if self.is_in_recovery_mode:
            rospy.loginfo("Stopping recovery behavior - target found or giving up")
            self.is_in_recovery_mode = False
            self.current_scan_index = 0
            self.recovery_start_time = None

            # Return head to center position
            self._look_centre_point()

            # reset the time of recovery to avoid stop detection
            self.last_movement_time = rospy.Time.now()

    def _check_target_recovery(self):
        """Check if target has been recovered during scanning"""
        current_time = rospy.Time.now()

        # If we have a recent good detection, target is recovered
        if current_time - self.good_detection_time < rospy.Duration(2.0):
            if self.is_in_recovery_mode:
                rospy.loginfo("Target recovered during scanning!")
                self._tts("Found you! Continuing to follow.", wait=False)
                self._stop_recovery_behavior()
                return True

        return False

    def follow(self) -> FollowResult:
        result = FollowResult()
        rate = rospy.Rate(10)
        self.last_movement_time = rospy.Time.now()
        self.good_detection_time = rospy.Time.now()
        previous_target = None
        just_started = True

        while not rospy.is_shutdown():
            # Check if target has been recovered during scanning
            if self._check_target_recovery():
                # Target was recovered, continue normal operation
                pass

            # Check if continues bad detection or start recovery behavior
            if self.good_detection_time + self.target_moving_timeout_duration < rospy.Time.now():
                if not self.is_in_recovery_mode:
                    rospy.loginfo("Tracking stopped for no good detection.")
                    self._tts("I cannot find you anymore.", wait=True)
                    self._start_recovery_behavior()
                else:
                    # Continue recovery behavior
                    if not self._update_recovery_behavior():
                        # Recovery timed out or failed
                        self._tts("I give up looking for you. Please come back.", wait=True)
                        break
            else:
                # Good detection - stop recovery if active
                if self.is_in_recovery_mode:
                    self._stop_recovery_behavior()

            # Skip normal navigation logic if in recovery mode
            if self.is_in_recovery_mode:
                rate.sleep()
                continue

            if not self.is_navigating and not just_started:
                # todo: need to consider recovery behavior
                # self._tts("Might stopped.", wait=True)
                if self.last_movement_time + self.target_moving_timeout_duration < rospy.Time.now():
                    rospy.loginfo("Tracking stopped for no movement.")
                    self._tts("Have we arrived? I will stop following.", wait=True)
                    break

            # Timeout check: no detection for 5 seconds
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
                    rospy.loginfo("Goal reached.")

            # Navigate to the next target
            if len(self.target_list) == 0:
                rospy.logwarn("No target list available.")
                rate.sleep()
                continue
            last_pose_in_list = self.target_list[-1]
            target_pose = None
            for i in reversed(range(len(self.target_list))):
                if self._target_boundary <= self._euclidian_distance(self.target_list[i], last_pose_in_list):
                    target_pose = self.target_list[i]
                    break
            if target_pose is None:
                rospy.logwarn("No target pose available.")
                rate.sleep()
                continue
            if previous_target and self._euclidian_distance(target_pose,
                                                            previous_target) < self._new_goal_threshold_min:
                rospy.logwarn("Target pose is the same as previous target pose.")
                rate.sleep()
                continue

            # Check if robot is already close enough to the target to avoid repeated navigation
            if not self.is_navigating:
                try:
                    robot_pose = self._robot_pose_in_odom()
                    distance_to_target = self._euclidian_distance(robot_pose.pose, target_pose)

                    # Check if target is too far away - ask person to wait
                    current_time = rospy.Time.now()
                    if distance_to_target > self._max_following_distance:
                        # Only send warning if enough time has passed since last warning
                        if current_time - self.last_distance_warning_time > self.distance_warning_interval:
                            rospy.loginfo(
                                f"Target too far ({distance_to_target:.2f}m > {self._max_following_distance}m) - asking person to wait")
                            self._tts("Please wait for me. You are too far away.", wait=False)
                            self.last_distance_warning_time = current_time

                        # Skip navigation if target is too far
                        # rate.sleep()
                        # continue

                    # If robot is already close enough to target, don't navigate
                    if distance_to_target < self._stopping_distance:
                        rospy.logdebug(
                            f"Robot already close to target ({distance_to_target:.2f}m < {self._stopping_distance}m), skipping navigation")
                        rate.sleep()
                        continue

                except Exception as e:
                    rospy.logwarn(f"Failed to get robot pose: {e}")
                    # Continue with navigation if we can't get robot pose

                # Set orientation to face the final pose, not the target pose
                goal_orientation = self._compute_face_quat(target_pose, last_pose_in_list)
                pose_with_orientation = Pose(
                    position=target_pose.position,
                    orientation=goal_orientation,
                )

                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "odom"
                pose_stamped.header.stamp = rospy.Time.now()
                pose_stamped.pose = pose_with_orientation

                goal_pose = self._tf_pose(pose_stamped, "map")
                rospy.loginfo(f"Setting navigation goal to intermediate point, facing final point")
                self._move_base(goal_pose)
                self.is_navigating = True
                just_started = False
                previous_target = target_pose  # Update previous target after setting new goal

            rate.sleep()
        return result


if __name__ == "__main__":
    follower = PersonFollower()
    follower.follow()
    rospy.spin()
