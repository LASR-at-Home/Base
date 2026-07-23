import rclpy
import yasmin
from yasmin_ros import ServiceState
import yasmin_ros
from time import sleep, time
import message_filters
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lasr_vision_interfaces.srv import YoloPoseDetection3D
from rclpy.duration import Duration
import tf2_ros as tf
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
import math
import numpy as np
import logging

logging.basicConfig(level=logging.INFO)


class DetectWavingPersonRGB(ServiceState):
    """
    Detect waving customers using YOLO 3D pose detection.

    Subscribes to RGB + depth images, detects hand-up poses, and returns
    the closest waving person's position as a PointStamped.

    Key detection logic:
    - Hand is considered "up" if wrist Z > shoulder Z (in 3D)
    - Returns closest waving person by distance from robot
    - Outputs: wave_detected (bool), wave_position (PointStamped), detection_confidence (float)
    """

    def __init__(
        self,
        image_topic: str = "/head_front_camera/rgb/image_raw",
        depth_image_topic: str = "/head_front_camera/depth/image_raw",
        depth_camera_info_topic: str = "/head_front_camera/depth/camera_info",
        model: str = "yolo11n-pose.pt",
        confidence: float = 0.5,
        target_frame: str = "map",
        slop: float = 0.5,
    ):
        super().__init__(
            srv_type=YoloPoseDetection3D,
            srv_name="/yolo/detect3d_pose",
            create_request_handler=self._create_req,
            outcomes=["waving", "not_waving", "failed"],
            response_handler=self._response_handler,
        )
        self.set_description("Detect waving customer via YOLO 3D pose detection")
        self.add_output_key("wave_detected")
        self.add_output_key("wave_position")
        self.add_output_key("detection_confidence")

        self.image_topic = image_topic
        self.depth_image_topic = depth_image_topic
        self.depth_camera_info_topic = depth_camera_info_topic
        self.model = model
        self.confidence = confidence
        self.target_frame = target_frame

        # Setup sensor synchronization
        camera_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Cache camera info separately (doesn't change often)
        self.cam_info = None
        self._node.create_subscription(
            CameraInfo,
            self.depth_camera_info_topic,
            self._cache_camera_info,
            qos_profile=camera_qos,
        )

        # Synchronize image and depth
        image_sub = message_filters.Subscriber(
            self._node, Image, self.image_topic, qos_profile=camera_qos
        )
        depth_sub = message_filters.Subscriber(
            self._node, Image, self.depth_image_topic, qos_profile=camera_qos
        )
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub], queue_size=30, slop=slop
        )
        self.data = None

        # Subscribe to laser scan for distance estimates
        self.latest_scan = None
        self._node.create_subscription(
            LaserScan,
            "/scan",
            self._laser_callback,
            10,
            # qos_profile=camera_qos,
        )

        # Publishers for visualization
        self._marker_publisher = self._node.create_publisher(
            MarkerArray,
            "/restaurant/waving_person_markers",
            10,
        )

        # Subscribe to robot pose in map frame
        self._robot_pose = None
        self._node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            lambda msg: setattr(self, "_robot_pose", msg.pose.pose),
            qos_profile=camera_qos,
        )

        # TF2 buffer for frame transformations
        self._tf_buffer = tf.Buffer()
        self._tf_listener = tf.TransformListener(self._tf_buffer, self._node)

    def _cache_camera_info(self, msg: CameraInfo) -> None:
        """Cache camera info once at startup"""
        if self.cam_info is None:
            self.cam_info = msg

    def _laser_callback(self, msg: LaserScan) -> None:
        """Store latest laser scan for fallback distance estimates"""
        self.latest_scan = msg

    def _create_req(self, blackboard):
        """Build service request with synchronized image/depth and cached camera info"""
        # Wait for camera info
        if self.cam_info is None:
            deadline = time() + 5.0
            while self.cam_info is None and time() < deadline:
                # rclpy.spin_once(self._node, timeout_sec=0.1)
                sleep(1)
            if self.cam_info is None:
                self._node.get_logger().error(
                    f"Timed out waiting for camera info on {self.depth_camera_info_topic}"
                )
                return None

        # Wait for synchronized image + depth
        self.data = None

        def callback(image_msg, depth_msg):
            if self.data is not None:
                return
            self.data = (image_msg, depth_msg)

        self.ts.registerCallback(callback)

        deadline = time() + 5.0
        while not self.data:
            if time() > deadline:
                self._node.get_logger().error(
                    f"Timed out waiting for synced rgb/depth frames on "
                    f"{self.image_topic} / {self.depth_image_topic}"
                )
                return None
            # rclpy.spin_once(self._node, timeout_sec=0.1)
            sleep(1)

        image_msg, depth_msg = self.data

        # Build service request
        req = YoloPoseDetection3D.Request()
        req.image_raw = image_msg
        req.depth_image = depth_msg
        req.depth_camera_info = self.cam_info
        req.model = self.model
        req.confidence = self.confidence
        req.target_frame = self.target_frame

        # Store laser scan and request timestamp for response handling
        self._request_scan = self.latest_scan
        self._request_timestamp = image_msg.header.stamp
        return req

    def _get_laser_distance_at_angle(self, scan: LaserScan, angle_rad: float) -> float:
        """Get distance from laser at a specific angle. Returns distance or None if invalid."""
        if scan is None:
            yasmin.YASMIN_LOG_DEBUG("  Laser scan is None")
            return None

        try:
            # Normalize angle to laser scan range
            angle_index = (angle_rad - scan.angle_min) / scan.angle_increment
            angle_index = int(round(angle_index))

            yasmin.YASMIN_LOG_DEBUG(
                f"  Laser lookup: angle={math.degrees(angle_rad):.1f}°, "
                f"angle_min={math.degrees(scan.angle_min):.1f}°, "
                f"angle_max={math.degrees(scan.angle_max):.1f}°, "
                f"angle_increment={math.degrees(scan.angle_increment):.3f}°, "
                f"angle_index={angle_index}, num_ranges={len(scan.ranges)}"
            )

            if 0 <= angle_index < len(scan.ranges):
                dist = scan.ranges[angle_index]
                yasmin.YASMIN_LOG_DEBUG(
                    f"  Raw laser range[{angle_index}] = {dist:.2f}m, "
                    f"valid=[{scan.range_min:.2f}, {scan.range_max:.2f}]"
                )
                if scan.range_min <= dist <= scan.range_max:
                    return dist
                else:
                    yasmin.YASMIN_LOG_DEBUG(f"  Distance {dist:.2f}m out of range")
            else:
                yasmin.YASMIN_LOG_DEBUG(
                    f"  Angle index {angle_index} out of bounds [0, {len(scan.ranges)})"
                )
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"Failed to get laser distance: {e}")
        return None

    def _calculate_person_center(self, keypoints_dict):
        """Calculate center of person from all keypoints."""
        valid_points = []
        for name, point in keypoints_dict.items():
            if not (math.isnan(point.x) or math.isnan(point.y) or math.isnan(point.z)):
                valid_points.append(point)

        if not valid_points:
            return None

        center = Point(
            x=sum(p.x for p in valid_points) / len(valid_points),
            y=sum(p.y for p in valid_points) / len(valid_points),
            z=sum(p.z for p in valid_points) / len(valid_points),
        )
        return center

    def _transform_to_base_footprint(self, point, timestamp):
        """Transform point from map frame to base_footprint frame using TF2.

        Returns (success: bool, transformed_point: Point).
        If transform fails, returns (False, None) — caller should not use the result.
        """
        try:
            # Lookup transform from map to base_footprint
            transform = self._tf_buffer.lookup_transform(
                "base_footprint", "map", timestamp, Duration(seconds=0.5)
            )
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(
                f"TF map->base_footprint lookup failed ({type(e).__name__}): {e}"
            )
            return False, None

        try:
            # Create PointStamped in map frame
            point_stamped = PointStamped()
            point_stamped.header.frame_id = "map"
            point_stamped.header.stamp = timestamp
            point_stamped.point = point

            # Transform to base_footprint frame
            point_transformed = do_transform_point(point_stamped, transform)
            return True, point_transformed.point
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(
                f"Point transformation failed ({type(e).__name__}): {e}"
            )
            return False, None

    def _response_handler(self, blackboard, response):
        """Process YOLO response and detect waving customers.

        Strategy:
        1. Use YOLO 3D detection if depth is valid (most accurate)
        2. Fall back to laser distance + RGB direction if depth unavailable
        """
        # Extract camera intrinsics for fallback calculation
        K = self.cam_info.k
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]

        # Find all people with hands up
        best_point = None
        best_dist = None
        best_confidence = 0
        best_method = None

        yasmin.YASMIN_LOG_INFO(f"Processing {len(response.detections)} detections")
        for det_idx, det in enumerate(response.detections):
            # Convert keypoints list to dict for easier access
            kp = {k.keypoint_name: k.point for k in det.keypoints}
            yasmin.YASMIN_LOG_DEBUG(f"Detection {det_idx}: has {len(kp)} keypoints")

            # Check for hands up (wrists above shoulders in Z)
            is_waving = True

            # Try left hand (wrist must be significantly above shoulder, not just barely)
            has_left_wrist = "left_wrist" in kp and "left_shoulder" in kp
            if has_left_wrist:
                wrist_z = kp["left_wrist"].z
                shoulder_z = kp["left_shoulder"].z
                is_up = wrist_z > shoulder_z + 0.1
                yasmin.YASMIN_LOG_DEBUG(
                    f"  Left hand: wrist_z={wrist_z:.3f}, shoulder_z={shoulder_z:.3f}, diff={wrist_z-shoulder_z:.3f}, is_up={is_up}"
                )
                if is_up:
                    is_waving = True

            # Try right hand (wrist must be significantly above shoulder, not just barely)
            if not is_waving:
                has_right_wrist = "right_wrist" in kp and "right_shoulder" in kp
                if has_right_wrist:
                    wrist_z = kp["right_wrist"].z
                    shoulder_z = kp["right_shoulder"].z
                    is_up = wrist_z > shoulder_z + 0.1
                    yasmin.YASMIN_LOG_DEBUG(
                        f"  Right hand: wrist_z={wrist_z:.3f}, shoulder_z={shoulder_z:.3f}, diff={wrist_z-shoulder_z:.3f}, is_up={is_up}"
                    )
                    if is_up:
                        is_waving = True

            if not is_waving:
                yasmin.YASMIN_LOG_DEBUG(f"  No hand-up detected, skipping")
                continue

            # Calculate center of person from all keypoints
            person_center = self._calculate_person_center(kp)
            if person_center is None:
                yasmin.YASMIN_LOG_INFO(
                    f"  ✗ Could not calculate person center (no valid keypoints), skipping"
                )
                continue

            yasmin.YASMIN_LOG_INFO(
                f"  ✓ Person center calculated: ({person_center.x:.3f}, {person_center.y:.3f}, {person_center.z:.3f})"
            )

            # Check for NaN or invalid values in depth estimate
            has_nan = (
                math.isnan(person_center.x)
                or math.isnan(person_center.y)
                or math.isnan(person_center.z)
            )

            # Primary: use 3D depth estimate if valid
            dist = math.sqrt(person_center.x**2 + person_center.y**2)
            method = "depth"
            point_to_use = person_center

            # Fallback: if depth seems unreliable (NaN, Z > 5m where depth error is large, or < 0.1m),
            # try laser-based estimate (laser is more reliable for distant objects)
            trigger_reason = None
            if has_nan:
                trigger_reason = "NaN in depth"
            elif person_center.z > 5.0:
                trigger_reason = f"depth > 5m (z={person_center.z:.2f}m)"
            elif person_center.z < 0.1:
                trigger_reason = f"depth < 0.1m (z={person_center.z:.2f}m)"

            if trigger_reason:
                yasmin.YASMIN_LOG_INFO(
                    f"Depth unreliable ({trigger_reason}), attempting laser fallback"
                )

                # Transform person center from map frame to base_footprint frame for laser query
                yasmin.YASMIN_LOG_DEBUG("  Attempting TF: map -> base_footprint")
                tf_success, point_in_base = self._transform_to_base_footprint(
                    person_center, self._request_timestamp
                )

                if not tf_success:
                    yasmin.YASMIN_LOG_WARN(
                        "  ✗ Cannot use laser fallback without TF (coordinates would be wrong frame)"
                    )
                else:
                    yasmin.YASMIN_LOG_DEBUG(
                        f"  ✓ TF successful: map ({person_center.x:.3f}, {person_center.y:.3f}) "
                        f"-> base_footprint ({point_in_base.x:.3f}, {point_in_base.y:.3f})"
                    )
                    angle_rad = math.atan2(point_in_base.y, point_in_base.x)
                    yasmin.YASMIN_LOG_DEBUG(
                        f"  Transformed to base_footprint: ({point_in_base.x:.2f}, {point_in_base.y:.2f})"
                    )
                    yasmin.YASMIN_LOG_DEBUG(
                        f"  Angle in base_footprint: {math.degrees(angle_rad):.1f}° (rad={angle_rad:.3f})"
                    )

                    if self._request_scan is None:
                        yasmin.YASMIN_LOG_WARN(
                            "  No laser scan available, cannot use fallback"
                        )
                    else:
                        laser_dist = self._get_laser_distance_at_angle(
                            self._request_scan, angle_rad
                        )

                        if laser_dist is not None:
                            yasmin.YASMIN_LOG_INFO(
                                f"  Laser distance: {laser_dist:.2f}m"
                            )
                            # Use laser distance but keep XY from depth (direction is reliable)
                            magnitude = math.sqrt(
                                person_center.x**2 + person_center.y**2
                            )
                            if magnitude > 0:
                                # Scale the depth estimate to match laser distance
                                scale = laser_dist / magnitude
                                point_to_use = Point(
                                    x=person_center.x * scale,
                                    y=person_center.y * scale,
                                    z=laser_dist
                                    * 0.2,  # Assume person height ~0.2m from ground estimate
                                )
                                dist = laser_dist
                                method = "laser"
                                yasmin.YASMIN_LOG_DEBUG(
                                    f"  Using laser fallback: scaled to ({point_to_use.x:.2f}, {point_to_use.y:.2f}, {point_to_use.z:.2f})"
                                )
                            else:
                                yasmin.YASMIN_LOG_WARN(
                                    "  Magnitude is zero, cannot scale"
                                )
                        else:
                            yasmin.YASMIN_LOG_WARN(
                                "  No laser distance found at that angle"
                            )

            # Select closest waving person
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best_point = point_to_use
                best_confidence = 0.5
                best_method = method

        if best_point is None:
            yasmin.YASMIN_LOG_INFO("✗ No waving customers detected")
            blackboard["wave_detected"] = False
            blackboard["wave_position"] = PointStamped()
            blackboard["detection_confidence"] = 0.0
            # Publish empty marker array to clear previous detections
            self._marker_publisher.publish(MarkerArray())
            return "not_waving"

        yasmin.YASMIN_LOG_INFO(
            f"✓ Waving customer detected (method={best_method}): "
            f"pos=({best_point.x:.2f}, {best_point.y:.2f}, {best_point.z:.2f})m, "
            f"dist={best_dist:.2f}m"
        )

        # Publish visualization markers
        marker_array = MarkerArray()

        # Marker 1: Detected person position (sphere)
        person_marker = Marker()
        person_marker.header.frame_id = self.target_frame
        person_marker.header.stamp = self._node.get_clock().now().to_msg()
        person_marker.id = 0
        person_marker.type = Marker.SPHERE
        person_marker.action = Marker.ADD
        person_marker.pose.position = best_point
        person_marker.scale.x = 0.3
        person_marker.scale.y = 0.3
        person_marker.scale.z = 0.3
        # Color by method: green=depth, cyan=laser
        if best_method == "depth":
            person_marker.color.r = 0.0
            person_marker.color.g = 1.0
            person_marker.color.b = 0.0
        else:  # laser
            person_marker.color.r = 0.0
            person_marker.color.g = 1.0
            person_marker.color.b = 1.0
        person_marker.color.a = 0.8
        marker_array.markers.append(person_marker)

        # Marker 2: Laser ray line (if laser fallback was used)
        if best_method == "laser" and self._robot_pose is not None:
            # Create line from robot position to detected person
            line_marker = Marker()
            line_marker.header.frame_id = self.target_frame
            line_marker.header.stamp = self._node.get_clock().now().to_msg()
            line_marker.id = 1
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.05  # line width
            line_marker.color.r = 0.0
            line_marker.color.g = 1.0
            line_marker.color.b = 1.0
            line_marker.color.a = 0.6
            # Line from robot position to person
            line_marker.points.append(self._robot_pose.position)
            line_marker.points.append(best_point)
            marker_array.markers.append(line_marker)

        self._marker_publisher.publish(marker_array)

        blackboard["wave_detected"] = True
        blackboard["wave_position"] = PointStamped(
            header=Header(frame_id=self.target_frame), point=best_point
        )
        blackboard["detection_confidence"] = best_confidence
        return "waving"


def main(args=None):
    """Test harness for DetectWavingPersonRGB state.

    Runs the detection in a loop for testing and visualization.
    Publish markers to /restaurant/waving_person_markers in RViz.
    """
    rclpy.init(args=args)
    yasmin_ros.set_ros_loggers()
    # Create state machine that loops detection
    sm = yasmin.StateMachine(
        outcomes=["succeeded", "failed"],
        handle_sigint=True,
    )
    sm.add_output_key("wave_detected")
    sm.add_output_key("wave_position")
    sm.add_output_key("detection_confidence")

    def loop_cb(blackboard):
        """After detection, loop back for continuous testing"""
        return "loop"

    # Add detection state
    sm.add_state(
        "DETECT",
        DetectWavingPersonRGB(
            image_topic="/head_front_camera/rgb/image_raw",
            depth_image_topic="/head_front_camera/depth/image_raw",
            depth_camera_info_topic="/head_front_camera/depth/camera_info",
            model="yolo11n-pose.pt",
            confidence=0.5,
            target_frame="map",
        ),
        transitions={
            "waving": "LOG_DETECTION",
            "not_waving": "LOG_NO_DETECTION",
            "failed": "LOG_FAILED",
            "aborted": "LOG_FAILED",  # ServiceState returns aborted on service failure
        },
    )

    # Log detection
    def log_detection(bb):
        if "wave_position" in bb:
            pos = bb["wave_position"].point
            yasmin.YASMIN_LOG_INFO(
                f"✓ Detected waving person at ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})"
            )
        return "loop"

    sm.add_state(
        "LOG_DETECTION",
        yasmin.CbState(["loop"], log_detection),
        transitions={"loop": "DETECT"},
    )

    # Log no detection
    def log_no_detection(bb):
        yasmin.YASMIN_LOG_INFO("✗ No waving customers detected")
        return "loop"

    sm.add_state(
        "LOG_NO_DETECTION",
        yasmin.CbState(["loop"], log_no_detection),
        transitions={"loop": "DETECT"},
    )

    # Log failure
    def log_failed(bb):
        yasmin.YASMIN_LOG_ERROR("✗ Detection failed")
        return "loop"

    sm.add_state(
        "LOG_FAILED",
        yasmin.CbState(["loop"], log_failed),
        transitions={"loop": "DETECT"},
    )

    yasmin.YASMIN_LOG_INFO(
        "Starting DetectWavingPersonRGB test loop...\n"
        "Visualize with RViz: /restaurant/waving_person_markers\n"
        "Press Ctrl+C to exit"
    )

    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(f"State machine ended with outcome: {outcome}")
    except KeyboardInterrupt:
        yasmin.YASMIN_LOG_INFO("Test interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
