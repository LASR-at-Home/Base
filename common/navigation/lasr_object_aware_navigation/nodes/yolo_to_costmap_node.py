
#!/usr/bin/env python3
import rospy
import rosparam
import numpy as np
import math

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from tf2_geometry_msgs import do_transform_point

from lasr_vision_msgs.srv import YoloDetection3D, YoloDetection3DRequest

import tf2_ros
import tf2_geometry_msgs
import cv2

# COCO class names for YOLO detection
COCO_CLASSES = [
    "brush",
    "cloth",
    "polish",
    "sponge",
    "bowl",
    "cup",
    "fork",
    "knife",
    "plate",
    "spoon",
    "coffee",
    "coke",
    "fanta",
    "kuat",
    "milk",
    "orange_juice",
    "broth",
    "broth_box",
    "corn_flour",
    "ketchup",
    "mayo",
    "oats",
    "tuna",
    "apple",
    "lemon",
    "lime",
    "pear",
    "tangerine",
    "cheese_snack",
    "chocolate_bar",
    "cornflakes",
    "crisps",
    "gum_balls",
    "peanuts",
    "pringles",
    "bag",
    "dishwasher_tab",
    "dishwasher_tab_bag",
    "cornflakes_container"
]


class ObstacleInfo:
    """Data structure to hold obstacle information in map coordinate frame"""

    def __init__(self, x, y, z, radius, label, detection_frame_id):
        self.x = x  # X position in map frame
        self.y = y  # Y position in map frame
        self.z = z  # Z position in map frame
        self.radius = radius  # Obstacle radius for hexagon generation
        self.label = label  # Object class label
        self.detection_frame_id = detection_frame_id  # Frame ID when first detected
        self.last_seen_frame = detection_frame_id  # Last frame when obstacle was seen
        self.confidence = 1.0  # Detection confidence


class MapObstacleManager:
    """Manages detected obstacles in map frame with frame-based aging mechanism"""

    def __init__(self, max_unseen_frames=10, matching_threshold=0.5):
        self.obstacles = {}  # Dictionary: obstacle_id -> ObstacleInfo
        self.current_frame_id = 0  # Current detection frame counter
        self.max_unseen_frames = max_unseen_frames  # Maximum frames before removing obstacle
        self.matching_threshold = matching_threshold  # Distance threshold for matching obstacles
        self.next_obstacle_id = 0  # Counter for generating unique obstacle IDs

    def update_obstacles(self, new_detections):
        """Update obstacle map with new detections from YOLO"""
        self.current_frame_id += 1

        # Process each new detection
        for detection in new_detections:
            if self.is_valid_detection(detection):
                self.add_or_update_obstacle(detection)

        # Remove obstacles that haven't been seen for too long
        self.cleanup_old_obstacles()

        rospy.loginfo_throttle(2.0, f"[ObstacleManager] Frame {self.current_frame_id}: "
                                    f"{len(self.obstacles)} active obstacles")

    def is_valid_detection(self, detection):
        """Validate detection data for NaN values and reasonable coordinate range"""
        if not hasattr(detection, 'point') or detection.point is None:
            return False

        # Check for NaN values in coordinates
        if (math.isnan(detection.point.x) or
                math.isnan(detection.point.y) or
                math.isnan(detection.point.z)):
            rospy.logwarn_throttle(1.0, "Detection contains NaN values, skipping")
            return False

        # Check if coordinates are within reasonable range
        if (abs(detection.point.x) > 100 or
                abs(detection.point.y) > 100 or
                abs(detection.point.z) > 10):
            rospy.logwarn_throttle(1.0, "Detection out of reasonable range, skipping")
            return False

        return True

    def find_matching_obstacle(self, new_detection):
        """Find existing obstacle that matches the new detection based on distance"""
        for obs_id, obstacle in self.obstacles.items():
            distance = math.sqrt(
                (obstacle.x - new_detection.point.x) ** 2 +
                (obstacle.y - new_detection.point.y) ** 2
            )
            if distance < self.matching_threshold:
                return obs_id
        return None

    def add_or_update_obstacle(self, detection):
        """Add new obstacle or update existing obstacle position"""
        matching_id = self.find_matching_obstacle(detection)

        if matching_id:
            # Update existing obstacle with smoothed position
            obstacle = self.obstacles[matching_id]
            obstacle.last_seen_frame = self.current_frame_id

            # Apply exponential smoothing filter for position update
            alpha = 0.7  # Smoothing factor (0.0 = no update, 1.0 = full update)
            obstacle.x = alpha * obstacle.x + (1 - alpha) * detection.point.x
            obstacle.y = alpha * obstacle.y + (1 - alpha) * detection.point.y
            obstacle.z = alpha * obstacle.z + (1 - alpha) * detection.point.z
        else:
            # Add new obstacle with unique ID
            new_id = f"vo_det_{self.next_obstacle_id:04d}"
            self.next_obstacle_id += 1

            label = getattr(detection, "name", "unknown")

            self.obstacles[new_id] = ObstacleInfo(
                detection.point.x, detection.point.y, detection.point.z,
                0.25,  # Default radius, will be updated by main node
                label, self.current_frame_id
            )

    def cleanup_old_obstacles(self):
        """Remove obstacles that haven't been seen for too many frames"""
        to_remove = []
        for obs_id, obstacle in self.obstacles.items():
            frames_unseen = self.current_frame_id - obstacle.last_seen_frame
            if frames_unseen > self.max_unseen_frames:
                to_remove.append(obs_id)

        for obs_id in to_remove:
            del self.obstacles[obs_id]

        if to_remove:
            rospy.loginfo(f"[ObstacleManager] Removed {len(to_remove)} old obstacles")

    def get_obstacles_dict(self):
        """Get obstacles formatted as dictionary for VO layer parameter update"""
        vo_dict = {}
        point_id = 0

        for obs_id, obstacle in self.obstacles.items():
            # Generate hexagon polygon points around obstacle center
            hexagon_points = self.generate_hexagon_points(obstacle.x, obstacle.y, obstacle.radius)

            # Create unique label for this specific obstacle instance
            # Use obstacle ID to make each physical object have a unique label
            unique_label = f"{obstacle.label}_{obs_id}"

            # Create individual VO parameters for each hexagon vertex
            # Each vertex becomes a separate point with the same unique label
            for i in range(0, len(hexagon_points), 2):  # Step by 2 since points are [x1,y1,x2,y2,...]
                vertex_x = hexagon_points[i]
                vertex_y = hexagon_points[i + 1]

                # Create unique point ID
                point_vo_id = f"vo_{point_id:04d}"
                point_id += 1

                # Format: ["submap_id", "unique_label", x, y, z]
                vo_params = ["submap_0", unique_label, vertex_x, vertex_y, 0.0]
                vo_dict[point_vo_id] = vo_params

        return vo_dict

    def generate_hexagon_points(self, center_x, center_y, radius):
        """Generate hexagon vertices around center point with given radius"""
        points = []
        for i in range(6):
            angle = i * math.pi / 3  # 60 degrees between vertices
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            points.extend([x, y])  # Add x,y pair to flat list
        return points


class YoloToCostmapNode:
    """Main node that detects objects using YOLO and adds them as hexagonal obstacles to costmap"""

    def __init__(self):
        # Basic detection parameters
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.obstacle_radius = rospy.get_param("~obstacle_radius", 0.15)
        self.yolo_service = rospy.get_param("~yolo_service", "/yolo/detect3d")
        # self.yolo_model = rospy.get_param("~yolo_model", "yolo11l-seg.pt")
        self.yolo_model = rospy.get_param("~yolo_model", "last.pt")
        self.confidence = rospy.get_param("~confidence", 0.15)
        # self.class_filter = rospy.get_param("~filter", COCO_CLASSES)
        self.class_filter = rospy.get_param("~filter", COCO_CLASSES)  #["banana"])
        self.detection_interval = rospy.get_param("~interval", 1.0)  # Detection frequency in seconds

        # Virtual obstacle layer parameters
        self.vo_param_namespace = rospy.get_param("~vo_namespace", "mmap/vo/submap_0")

        # Object filtering parameters
        self.ignore_labels = set([label.lower() for label in rospy.get_param("~ignore_labels", ["bag"])])
        self.max_height = rospy.get_param("~max_height", 0.2)  # Maximum object height to consider

        # Camera and sensor topics
        self.image_topic = rospy.get_param("~image_topic", "/xtion/rgb/image_raw")
        self.depth_topic = rospy.get_param("~depth_image_topic", "/xtion/depth_registered/image_raw")
        self.camera_info_topic = rospy.get_param("~depth_camera_info_topic", "/xtion/depth_registered/camera_info")

        # Initialize obstacle manager with shorter lifespan for VO obstacles
        self.obstacle_manager = MapObstacleManager(
            max_unseen_frames=60,
            matching_threshold=self.obstacle_radius
        )

        # ROS communication components
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_depth = None
        self.latest_info = None

        # Subscribe to camera topics
        rospy.Subscriber(self.image_topic, Image, self.cb_image, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self.cb_depth, queue_size=1)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.cb_info, queue_size=1)

        # Publisher for visualization markers
        self.marker_pub = rospy.Publisher("/detected_objects_markers", MarkerArray, queue_size=1)

        # TF2 listener for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Wait for YOLO 3D detection service
        rospy.loginfo("Waiting for YOLO service: %s...", self.yolo_service)
        rospy.wait_for_service(self.yolo_service)
        self.yolo_srv = rospy.ServiceProxy(self.yolo_service, YoloDetection3D)
        rospy.loginfo("YOLO 3D service ready.")

        # Start detection timer
        self.timer = rospy.Timer(rospy.Duration(self.detection_interval), self.timer_cb)

        rospy.loginfo("YoloToCostmapNode initialized successfully")

    def cb_image(self, msg):
        """Callback to store latest RGB image"""
        self.latest_image = msg

    def cb_depth(self, msg):
        """Callback to store latest depth image"""
        self.latest_depth = msg

    def cb_info(self, msg):
        """Callback to store latest camera info"""
        self.latest_info = msg

    def timer_cb(self, event):
        """Main timer callback for YOLO detection and VO layer update"""
        if self.latest_image is None or self.latest_depth is None or self.latest_info is None:
            rospy.logwarn_throttle(10.0, "Waiting for image/depth/camera_info...")
            return

        try:
            # Call YOLO 3D detection service
            detections = self.call_yolo_service()
            # Process detections and update virtual obstacles
            self.process_detections(detections)
        except Exception as e:
            rospy.logwarn("Failed to get YOLO detections or process them: %s", str(e))

    def call_yolo_service(self):
        """Call YOLO 3D detection service with current sensor data"""
        req = YoloDetection3DRequest()
        req.model = self.yolo_model
        req.confidence = self.confidence
        req.filter = self.class_filter
        req.image_raw = self.latest_image
        req.depth_image = self.latest_depth
        req.depth_camera_info = self.latest_info
        req.target_frame = self.target_frame
        resp = self.yolo_srv(req)
        return resp.detected_objects

    def process_detections(self, detections):
        """Process YOLO detections and update VO layer with hexagonal obstacles"""
        # Filter detections based on various criteria
        filtered_detections = self.filter_detections(detections)

        # Update obstacle manager with filtered detections
        self.obstacle_manager.update_obstacles(filtered_detections)

        # Update obstacle radius for all managed obstacles
        for obstacle in self.obstacle_manager.obstacles.values():
            obstacle.radius = self.obstacle_radius

        # Update VO layer parameters with current obstacles
        self.update_vo_layer()

        # Publish visualization markers for RViz (including hexagon vertices)
        self.publish_markers()

    def filter_detections(self, detections):
        """Filter detections based on label blacklist and height constraints"""
        filtered = []

        # Convert depth image for height calculations
        try:
            depth_cv = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding="passthrough")
        except Exception as e:
            rospy.logwarn("Failed to decode depth image: %s", str(e))
            return []

        # Extract camera intrinsic parameters
        K = self.latest_info.K
        fx, fy = K[0], K[4]  # Focal lengths
        cx, cy = K[2], K[5]  # Principal point

        for idx, det in enumerate(detections):
            label = getattr(det, "name", "").lower()
            info_msg = f"[{idx}] Class: '{label}' "

            # Skip objects in ignore list
            if label in self.ignore_labels:
                rospy.loginfo(info_msg + "Skipped: in ignore_labels.")
                continue

            # Skip detections without valid 3D point
            if not hasattr(det, "point") or det.point is None:
                rospy.logwarn(info_msg + "Skipped: No valid 3D point.")
                continue

            # Apply height filtering using object segmentation mask
            if self.max_height > 0:
                real_height = self.calculate_real_height(det, depth_cv, fx, fy, cx, cy)
                if real_height is not None and real_height > self.max_height:
                    rospy.loginfo(info_msg + f"Skipped: height {real_height:.3f} > max_height={self.max_height:.2f}")
                    continue

            filtered.append(det)
            rospy.loginfo(info_msg + f"at ({det.point.x:.2f}, {det.point.y:.2f}) -- Added.")

        rospy.loginfo(f"[Detection Filter] Total: {len(detections)}, Filtered: {len(filtered)}")
        return filtered

    def calculate_real_height(self, detection, depth_cv, fx, fy, cx, cy):
        """Calculate real world height of detected object using segmentation mask"""
        if not hasattr(detection, "xyseg") or not detection.xyseg or len(detection.xyseg) < 6:
            return None

        try:
            # Convert segmentation points to contour
            contour = np.array(detection.xyseg, dtype=np.int32).reshape(-1, 2)
            mask = np.zeros(depth_cv.shape[:2], dtype=np.uint8)
            cv2.fillPoly(mask, [contour], color=255)

            # Extract depth values within segmentation mask
            indices = np.argwhere(mask == 255)
            if indices.shape[0] == 0:
                return None

            us = indices[:, 1]  # u coordinates (image columns)
            vs = indices[:, 0]  # v coordinates (image rows)
            zs = depth_cv[vs, us]  # Depth values
            valid = (zs > 0) & (~np.isnan(zs))  # Filter valid depth values

            if not np.any(valid):
                return None

            # Use median depth for more robust height calculation
            valid_us = us[valid]
            valid_vs = vs[valid]
            valid_zs = zs[valid]

            median_idx = np.argsort(valid_zs)[len(valid_zs) // 2]
            u_med = valid_us[median_idx]
            v_med = valid_vs[median_idx]
            z_med = valid_zs[median_idx]

            # Convert from image coordinates to camera coordinates
            x_cam = (u_med - cx) * z_med / fx
            y_cam = (v_med - cy) * z_med / fy

            # Transform to map frame to get absolute height
            pt_cam = PointStamped()
            pt_cam.header = self.latest_depth.header
            pt_cam.point = Point(x_cam, y_cam, z_med)

            pt_map = self.tf_buffer.transform(pt_cam, self.target_frame, rospy.Duration(0.25))
            return abs(pt_map.point.z)

        except Exception as e:
            rospy.logwarn(f"Error calculating real height: {e}")
            return None

    def update_vo_layer(self):
        """Update VO layer parameters using rosparam.upload_params and trigger reload"""
        try:
            # Get current obstacles as parameter dictionary
            vo_dict = self.obstacle_manager.get_obstacles_dict()

            # Clear existing VO parameters under namespace
            try:
                existing_params = rospy.get_param(self.vo_param_namespace, {})
                for param_name in existing_params.keys():
                    if param_name.startswith('vo_'):
                        full_param_path = f"{self.vo_param_namespace}/{param_name}"
                        rospy.delete_param(full_param_path)
                        rospy.logdebug(f"Deleted parameter: {full_param_path}")
            except Exception as e:
                rospy.logwarn(f"Error clearing old VO parameters: {e}")

            # Upload new VO parameters using rosparam.upload_params
            for vo_id, params in vo_dict.items():
                param_path = f"{self.vo_param_namespace}/{vo_id}"
                rosparam.upload_params(param_path, params)
                rospy.logdebug(f"Uploaded parameter {param_path}: {params}")

            rospy.loginfo(f"[VO Layer] Updated {len(vo_dict)} hexagonal obstacles using upload_params")

        except Exception as e:
            rospy.logwarn(f"Failed to update VO layer: {e}")
            import traceback
            rospy.logwarn(traceback.format_exc())

    def publish_markers(self):
        """Publish visualization markers for detected objects and hexagon vertices in RViz"""
        marker_array = MarkerArray()
        marker_id = 0

        # Iterate through all managed obstacles (not just current detections)
        for obs_id, obstacle in self.obstacle_manager.obstacles.items():
            label = obstacle.label

            # Create cylinder marker to represent hexagonal obstacle center
            center_marker = Marker()
            center_marker.header.frame_id = self.target_frame
            center_marker.header.stamp = rospy.Time.now()
            center_marker.ns = "obstacle_centers"
            center_marker.id = marker_id
            center_marker.type = Marker.CYLINDER
            center_marker.action = Marker.ADD

            center_marker.pose.position.x = obstacle.x
            center_marker.pose.position.y = obstacle.y
            center_marker.pose.position.z = obstacle.z
            center_marker.pose.orientation.w = 1.0

            center_marker.scale.x = self.obstacle_radius * 2
            center_marker.scale.y = self.obstacle_radius * 2
            center_marker.scale.z = 0.1

            # Orange color for obstacle center
            center_marker.color.a = 0.7
            center_marker.color.r = 1.0
            center_marker.color.g = 0.5
            center_marker.color.b = 0.0

            center_marker.lifetime = rospy.Duration(self.detection_interval * 3)
            marker_array.markers.append(center_marker)
            marker_id += 1

            # Create text marker for object label
            text_marker = Marker()
            text_marker.header.frame_id = self.target_frame
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "obstacle_labels"
            text_marker.id = marker_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = obstacle.x
            text_marker.pose.position.y = obstacle.y
            text_marker.pose.position.z = obstacle.z + 0.3
            text_marker.pose.orientation.w = 1.0

            text_marker.text = f"{label} ({obs_id})"
            text_marker.scale.z = 0.2

            # White color for text
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0

            text_marker.lifetime = rospy.Duration(self.detection_interval * 3)
            marker_array.markers.append(text_marker)
            marker_id += 1

            # Generate hexagon vertices for visualization
            hexagon_points = self.obstacle_manager.generate_hexagon_points(
                obstacle.x, obstacle.y, obstacle.radius
            )

            # Create small sphere markers for each hexagon vertex
            for i in range(0, len(hexagon_points), 2):  # Step by 2 since points are [x1,y1,x2,y2,...]
                vertex_x = hexagon_points[i]
                vertex_y = hexagon_points[i + 1]

                vertex_marker = Marker()
                vertex_marker.header.frame_id = self.target_frame
                vertex_marker.header.stamp = rospy.Time.now()
                vertex_marker.ns = "hexagon_vertices"
                vertex_marker.id = marker_id
                vertex_marker.type = Marker.SPHERE
                vertex_marker.action = Marker.ADD

                vertex_marker.pose.position.x = vertex_x
                vertex_marker.pose.position.y = vertex_y
                vertex_marker.pose.position.z = obstacle.z
                vertex_marker.pose.orientation.w = 1.0

                vertex_marker.scale.x = 0.05  # Small sphere size
                vertex_marker.scale.y = 0.05
                vertex_marker.scale.z = 0.05

                # Red color for hexagon vertices
                vertex_marker.color.a = 1.0
                vertex_marker.color.r = 1.0
                vertex_marker.color.g = 0.0
                vertex_marker.color.b = 0.0

                vertex_marker.lifetime = rospy.Duration(self.detection_interval * 3)
                marker_array.markers.append(vertex_marker)
                marker_id += 1

            # Create LINE_STRIP marker to draw hexagon outline
            hexagon_outline = Marker()
            hexagon_outline.header.frame_id = self.target_frame
            hexagon_outline.header.stamp = rospy.Time.now()
            hexagon_outline.ns = "hexagon_outlines"
            hexagon_outline.id = marker_id
            hexagon_outline.type = Marker.LINE_STRIP
            hexagon_outline.action = Marker.ADD

            hexagon_outline.pose.orientation.w = 1.0

            # Add hexagon vertices as points (need to close the loop)
            for i in range(0, len(hexagon_points), 2):
                point = Point()
                point.x = hexagon_points[i]
                point.y = hexagon_points[i + 1]
                point.z = obstacle.z
                hexagon_outline.points.append(point)

            # Close the hexagon by adding the first point again
            if len(hexagon_points) >= 2:
                point = Point()
                point.x = hexagon_points[0]
                point.y = hexagon_points[1]
                point.z = obstacle.z
                hexagon_outline.points.append(point)

            hexagon_outline.scale.x = 0.02  # Line width

            # Blue color for hexagon outline
            hexagon_outline.color.a = 1.0
            hexagon_outline.color.r = 0.0
            hexagon_outline.color.g = 0.0
            hexagon_outline.color.b = 1.0

            hexagon_outline.lifetime = rospy.Duration(self.detection_interval * 3)
            marker_array.markers.append(hexagon_outline)
            marker_id += 1

        # Publish all markers
        self.marker_pub.publish(marker_array)

        rospy.loginfo(f"[Markers] Published {len(marker_array.markers)} visualization markers")


if __name__ == "__main__":
    rospy.init_node("yolo_to_costmap_node")
    node = YoloToCostmapNode()
    rospy.spin()
