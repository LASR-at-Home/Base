#!/usr/bin/env python3

import rospy
import rosparam
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

import open3d as o3d
from sklearn.decomposition import PCA
import tf2_ros
import tf.transformations as tft
import time


def estimate_radius(eigen_vals: np.ndarray) -> float:
    """Estimate rope radius from PCA eigenvalues"""
    return float(np.sqrt(eigen_vals[1]))


class TrackedBarrier:
    """Simple tracked barrier object for stability with enhanced frame-based tracking"""

    def __init__(self, center, direction, length, points):
        self.center = center.copy()
        self.direction = direction.copy()
        self.length = length
        self.points = points.copy()
        self.confidence = 0.3  # 更高的初始置信度
        self.last_seen = time.time()
        self.detection_count = 1
        self.consecutive_detections = 1  # Track consecutive detections
        self.consecutive_misses = 0  # Track consecutive misses
        self.stable_id = None
        self.is_published = False  # Track if this barrier is being published

        # 增加强记忆机制
        self.peak_confidence = 0.3  # 记录历史最高置信度
        self.total_observation_time = 0.0  # 总观测时间
        self.first_seen = time.time()  # 首次观测时间

        # VO layer integration
        self.vo_point_ids = []  # 存储分配给该障碍物的VO点ID

        # Ground projection attributes
        self.ground_projection = None
        self.ground_endpoints = None
        self._update_ground_projection()

    def _update_ground_projection(self):
        """Update ground projection based on current 3D line"""
        if len(self.points) < 2:
            return

        # Project all points to ground (z=0)
        ground_points = self.points.copy()
        ground_points[:, 2] = 0.0

        # Find endpoints of ground projection
        if len(ground_points) >= 2:
            # Use first and last points as endpoints
            self.ground_endpoints = np.array([ground_points[0], ground_points[-1]])

            # Calculate center and direction of ground projection
            self.ground_projection = {
                'center': np.mean(self.ground_endpoints, axis=0),
                'direction': self.ground_endpoints[-1] - self.ground_endpoints[0],
                'length': np.linalg.norm(self.ground_endpoints[-1] - self.ground_endpoints[0])
            }

            # Normalize direction
            if self.ground_projection['length'] > 0:
                self.ground_projection['direction'] /= self.ground_projection['length']

    def update(self, new_center, new_direction, new_length, new_points):
        """Update barrier with new detection using weighted average"""
        # 更快的学习率，更快收敛
        alpha = 0.3  # 增加学习率

        self.center = (1 - alpha) * self.center + alpha * new_center
        self.direction = (1 - alpha) * self.direction + alpha * new_direction
        self.length = (1 - alpha) * self.length + alpha * new_length
        self.points = new_points  # Use latest points for visualization

        # Update ground projection
        self._update_ground_projection()

        # Update tracking counters
        self.consecutive_detections += 1
        self.consecutive_misses = 0
        self.detection_count += 1
        self.last_seen = time.time()

        # 更快的置信度增长
        confidence_boost = 0.15  # 增加置信度提升
        self.confidence = min(1.0, self.confidence + confidence_boost)

        # 更新历史最高置信度
        self.peak_confidence = max(self.peak_confidence, self.confidence)

        # 更新总观测时间
        self.total_observation_time += time.time() - self.first_seen

    def decay(self):
        """Reduce confidence when not detected - 使用更温和的衰减"""
        self.consecutive_misses += 1
        self.consecutive_detections = 0

        # 基于历史置信度的温和衰减
        if self.peak_confidence > 0.8:  # 如果历史上置信度很高
            self.confidence *= 0.95  # 非常温和的衰减
        elif self.peak_confidence > 0.6:
            self.confidence *= 0.90  # 温和的衰减
        else:
            self.confidence *= 0.85  # 标准衰减

    def is_similar(self, center, direction, distance_thresh=1.5, angle_thresh=0.6):
        """Check if new detection is similar to this tracked barrier - 放宽匹配条件"""
        # Check center distance - 放宽距离阈值
        center_dist = np.linalg.norm(self.center - center)
        if center_dist > distance_thresh:
            return False

        # Check direction similarity (cosine similarity) - 放宽角度阈值
        cos_angle = np.abs(np.dot(self.direction, direction))
        if cos_angle < np.cos(angle_thresh):  # angle_thresh in radians
            return False

        return True

    def should_start_publishing(self, min_consecutive_detections=2, min_confidence=0.3):
        """Check if barrier should start being published - 更宽松的发布条件"""
        return (self.consecutive_detections >= min_consecutive_detections and
                self.confidence >= min_confidence)

    def should_stop_publishing(self, max_consecutive_misses=120, min_confidence=0.1):
        """Check if barrier should stop being published - 更宽松的保持条件"""
        # 基于历史置信度的动态阈值
        if self.peak_confidence > 0.8:
            effective_max_misses = max_consecutive_misses * 2  # 高置信度障碍物保持更久
            effective_min_confidence = min_confidence * 0.5
        else:
            effective_max_misses = max_consecutive_misses
            effective_min_confidence = min_confidence

        return (self.consecutive_misses >= effective_max_misses or
                self.confidence < effective_min_confidence)

    def should_delete(self, max_age=60.0, min_confidence=0.02):
        """Check if barrier should be deleted completely - 非常宽松的删除条件"""
        age = time.time() - self.last_seen

        # 基于历史置信度的动态删除阈值
        if self.peak_confidence > 0.8:
            effective_max_age = max_age * 3  # 高置信度障碍物保存更久
            effective_min_confidence = min_confidence * 0.3
        elif self.peak_confidence > 0.6:
            effective_max_age = max_age * 2
            effective_min_confidence = min_confidence * 0.5
        else:
            effective_max_age = max_age
            effective_min_confidence = min_confidence

        return age > effective_max_age or self.confidence < effective_min_confidence


class DepthBarrierDetector:
    def __init__(self):
        # ROS parameters
        self.height_min = rospy.get_param("~height_min", 0.30)
        self.height_max = rospy.get_param("~height_max", 2.50)
        self.rope_r_min = rospy.get_param("~rope_radius_min", 0.001)
        self.rope_r_max = rospy.get_param("~rope_radius_max", 0.30)
        self.voxel_size = rospy.get_param("~voxel_size", 0.01)
        self.db_eps = rospy.get_param("~db_eps", 0.03)
        self.db_min_pts = rospy.get_param("~db_min_pts", 20)  # 减少最小点数，更容易检测
        self.eigen_ratio = rospy.get_param("~eigen_ratio", 8.0)  # 放宽线性度要求
        self.marker_w = rospy.get_param("~marker_width", 0.01)
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.range_min = rospy.get_param("~range_min", 0.30)
        self.range_max = rospy.get_param("~range_max", 5.00)

        # Enhanced tracking parameters - 更宽松的关联条件
        self.association_distance = rospy.get_param("~association_distance", 1.5)
        self.association_angle = rospy.get_param("~association_angle", 0.6)

        # Publishing control parameters - 更快的发布，更长的记忆
        self.min_consecutive_detections = rospy.get_param("~min_consecutive_detections", 2)
        self.min_confidence_to_publish = rospy.get_param("~min_confidence_to_publish", 0.3)
        self.max_consecutive_misses = rospy.get_param("~max_consecutive_misses", 120)
        self.min_confidence_to_keep = rospy.get_param("~min_confidence_to_keep", 0.1)

        # Deletion parameters - 更长的记忆保持
        self.max_age = rospy.get_param("~max_age", 60.0)
        self.min_confidence_to_exist = rospy.get_param("~min_confidence_to_exist", 0.02)

        # Ground projection parameters
        self.ground_z = rospy.get_param("~ground_z", 0.0)
        self.ground_margin = rospy.get_param("~ground_margin", 0.2)  # Extra margin around ground projection

        # Scanning frequency control - 更高的扫描频率
        self.scan_frequency = rospy.get_param("~scan_frequency", 2.0)  # 增加到2Hz
        self.last_scan_time = 0.0
        self.scan_interval = 1.0 / self.scan_frequency if self.scan_frequency > 0 else 0.0
        self.latest_image_msg = None  # Store latest image for controlled processing

        # VO layer parameters
        self.vo_namespace = rospy.get_param("~vo_namespace", "mmap/vo/submap_0")
        self.vo_update_enabled = rospy.get_param("~vo_update_enabled", True)
        self.vo_update_frequency = rospy.get_param("~vo_update_frequency", 2.0)  # 增加VO更新频率

        # VO point management - 修复关键问题
        self.vo_global_counter = 0  # 全局计数器，只增不减
        self.vo_assigned_ids = {}  # 映射 barrier_id -> [vo_point_ids]

        # ROS communication
        self.bridge = CvBridge()
        self.intrinsic = None
        self.camera_frame = None

        # TF components
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Tracking state
        self.tracked_barriers = []
        self.next_stable_id = 0
        self.published_marker_ids = set()  # Track published markers for cleanup

        # Publishers
        self.marker_pub = rospy.Publisher("barrier_marker_array", MarkerArray, queue_size=1)

        # Subscribers
        rospy.Subscriber("/xtion/depth_registered/camera_info", CameraInfo, self._cam_info_cb, queue_size=1)
        rospy.Subscriber("/xtion/depth_registered/image_raw", Image, self._image_store_cb, queue_size=1,
                         buff_size=2 ** 22)

        # Timer for controlled scanning
        if self.scan_frequency > 0:
            self.scan_timer = rospy.Timer(rospy.Duration(self.scan_interval), self._scan_timer_cb)
        else:
            # If frequency is 0 or negative, process every image (original behavior)
            rospy.Subscriber("/xtion/depth_registered/image_raw", Image, self._depth_cb, queue_size=1,
                             buff_size=2 ** 22)

        # Timer for VO layer updates
        if self.vo_update_enabled and self.vo_update_frequency > 0:
            vo_update_interval = 1.0 / self.vo_update_frequency
            self.vo_timer = rospy.Timer(rospy.Duration(vo_update_interval), self._vo_update_timer_cb)

        # Log parameters
        rospy.loginfo(f"Optimized tracking parameters for navigation:")
        rospy.loginfo(f"  - Scan frequency: {self.scan_frequency} Hz (faster detection)")
        rospy.loginfo(
            f"  - Min consecutive detections to publish: {self.min_consecutive_detections} (faster publishing)")
        rospy.loginfo(f"  - Max consecutive misses before unpublish: {self.max_consecutive_misses} (longer memory)")
        rospy.loginfo(f"  - Max age before deletion: {self.max_age}s (extended memory)")
        rospy.loginfo(f"  - Ground projections in frame: {self.target_frame}")
        rospy.loginfo(f"  - VO layer update enabled: {self.vo_update_enabled}")
        rospy.loginfo(f"  - VO layer update frequency: {self.vo_update_frequency} Hz")
        rospy.loginfo(f"  - VO namespace: {self.vo_namespace}")

    def _cam_info_cb(self, msg: CameraInfo):
        """Initialize camera intrinsics once"""
        if self.intrinsic is None:
            self.intrinsic = o3d.camera.PinholeCameraIntrinsic(
                msg.width, msg.height,
                msg.K[0], msg.K[4], msg.K[2], msg.K[5]
            )
            self.camera_frame = msg.header.frame_id or "xtion_depth_optical_frame"
            rospy.loginfo("Camera intrinsics received: frame = %s", self.camera_frame)

    def _image_store_cb(self, msg: Image):
        """Store the latest image for controlled processing"""
        self.latest_image_msg = msg

    def _scan_timer_cb(self, event):
        """Timer callback for controlled scanning frequency"""
        if self.latest_image_msg is not None:
            # Process the latest stored image
            self._depth_cb(self.latest_image_msg)

            # Update all unmatched barriers (decay their confidence)
            current_time = time.time()
            for barrier in self.tracked_barriers:
                # If barrier wasn't updated recently, decay it
                if current_time - barrier.last_seen > self.scan_interval * 1.5:
                    barrier.decay()

            # Clean up old barriers and update publishing status
            self._update_barrier_status()

            # Always publish markers to reflect current state
            self._publish_markers(self.latest_image_msg.header.stamp)

    def _vo_update_timer_cb(self, event):
        """Timer callback for VO layer updates"""
        if self.vo_update_enabled:
            self.update_vo_layer()

    def _allocate_vo_ids_for_barrier(self, barrier):
        """为barrier分配固定的VO点ID"""
        if barrier.stable_id not in self.vo_assigned_ids:
            # 分配两个连续的VO点ID
            point_id_1 = f"vo_{self.vo_global_counter:04d}"
            self.vo_global_counter += 1
            point_id_2 = f"vo_{self.vo_global_counter:04d}"
            self.vo_global_counter += 1

            self.vo_assigned_ids[barrier.stable_id] = [point_id_1, point_id_2]
            rospy.loginfo(f"Allocated VO IDs {point_id_1}, {point_id_2} for barrier {barrier.stable_id}")

        return self.vo_assigned_ids[barrier.stable_id]

    def _cleanup_vo_ids_for_barrier(self, barrier_id):
        """清理已删除barrier的VO点ID"""
        if barrier_id in self.vo_assigned_ids:
            old_ids = self.vo_assigned_ids[barrier_id]
            del self.vo_assigned_ids[barrier_id]
            rospy.loginfo(f"Cleaned up VO IDs {old_ids} for barrier {barrier_id}")
            return old_ids
        return []

    def get_barriers_dict(self):
        """Get barriers formatted as dictionary for VO layer parameter update"""
        vo_dict = {}

        for barrier in self.tracked_barriers:
            # Only include published barriers
            if not barrier.is_published or barrier.stable_id is None:
                continue

            # Only include barriers with valid ground projection
            if barrier.ground_endpoints is None or len(barrier.ground_endpoints) < 2:
                continue

            # Get or allocate VO IDs for this barrier
            vo_point_ids = self._allocate_vo_ids_for_barrier(barrier)

            # Create unique line label for this barrier
            line_label = f"line{barrier.stable_id}"

            # Add the two endpoints of the ground projection with fixed IDs
            for i, (point_id, endpoint) in enumerate(zip(vo_point_ids, barrier.ground_endpoints)):
                # Format: ["submap_0", "line_label", x, y, z]
                vo_params = ["submap_0", line_label, float(endpoint[0]), float(endpoint[1]), 0.0]
                vo_dict[point_id] = vo_params

        return vo_dict

    def update_vo_layer(self):
        """Update VO layer parameters using rosparam.upload_params"""
        try:
            # Get current barriers as parameter dictionary
            vo_dict = self.get_barriers_dict()

            # Get all currently published barrier IDs
            current_published_ids = set([b.stable_id for b in self.tracked_barriers if b.is_published])

            # Clean up VO IDs for barriers that are no longer published
            ids_to_cleanup = []
            for barrier_id in list(self.vo_assigned_ids.keys()):
                if barrier_id not in current_published_ids:
                    ids_to_cleanup.extend(self._cleanup_vo_ids_for_barrier(barrier_id))

            # Clear old VO parameters that are no longer needed
            try:
                existing_params = rospy.get_param(self.vo_namespace, {})
                for param_name in existing_params.keys():
                    if param_name.startswith('vo_'):
                        # Only delete if it's in our cleanup list or not in current dict
                        if param_name in ids_to_cleanup or param_name not in vo_dict:
                            full_param_path = f"{self.vo_namespace}/{param_name}"
                            rospy.delete_param(full_param_path)
                            rospy.logdebug(f"Deleted parameter: {full_param_path}")
            except Exception as e:
                rospy.logwarn(f"Error clearing old VO parameters: {e}")

            # Upload new/updated VO parameters
            for vo_id, params in vo_dict.items():
                param_path = f"{self.vo_namespace}/{vo_id}"
                rosparam.upload_params(param_path, params)
                rospy.logdebug(f"Uploaded parameter {param_path}: {params}")

            active_barriers = len([b for b in self.tracked_barriers if b.is_published])
            rospy.loginfo(f"[VO Layer] Updated {len(vo_dict)} endpoints from {active_barriers} active barriers")

        except Exception as e:
            rospy.logwarn(f"Failed to update VO layer: {e}")
            import traceback
            rospy.logwarn(traceback.format_exc())

    def _extract_line_features(self, cluster_pts):
        """Extract center, direction, and length from cluster points"""
        pca = PCA(n_components=3).fit(cluster_pts)

        center = np.mean(cluster_pts, axis=0)
        direction = pca.components_[0]

        # Ensure consistent direction (always point in positive direction)
        if direction[0] < 0:
            direction = -direction

        # Calculate length as distance between extreme points along principal axis
        projections = cluster_pts @ direction
        length = np.max(projections) - np.min(projections)

        return center, direction, length

    def _create_ground_projection_markers(self, barrier, stamp):
        """Create ground projection markers for a barrier in the target frame"""
        markers = []

        if barrier.ground_projection is None or barrier.ground_endpoints is None:
            return markers

        # Ground line marker
        ground_line = Marker()
        ground_line.header.stamp = stamp
        ground_line.header.frame_id = self.target_frame  # Ground projections are in target frame (usually map)
        ground_line.ns = "ground_projections"
        ground_line.id = barrier.stable_id
        ground_line.type = Marker.LINE_STRIP
        ground_line.action = Marker.ADD
        ground_line.scale.x = self.marker_w * 2  # Slightly thicker for ground projection
        ground_line.lifetime = rospy.Duration(0)

        # Red color for ground projection
        ground_line.color.r, ground_line.color.g, ground_line.color.b, ground_line.color.a = (1, 0, 0, 0.8)

        # Add points at ground level
        ground_line.points = [
            Point(x=float(barrier.ground_endpoints[0][0]),
                  y=float(barrier.ground_endpoints[0][1]),
                  z=float(self.ground_z)),
            Point(x=float(barrier.ground_endpoints[1][0]),
                  y=float(barrier.ground_endpoints[1][1]),
                  z=float(self.ground_z))
        ]
        markers.append(ground_line)

        # Ground projection area (optional - creates a rectangular area)
        if barrier.ground_projection['length'] > 0:
            area_marker = Marker()
            area_marker.header.stamp = stamp
            area_marker.header.frame_id = self.target_frame  # Ground areas are in target frame
            area_marker.ns = "ground_areas"
            area_marker.id = barrier.stable_id
            area_marker.type = Marker.CUBE
            area_marker.action = Marker.ADD
            area_marker.lifetime = rospy.Duration(0)

            # Position at center of ground projection
            area_marker.pose.position.x = float(barrier.ground_projection['center'][0])
            area_marker.pose.position.y = float(barrier.ground_projection['center'][1])
            area_marker.pose.position.z = float(self.ground_z + 0.01)  # Slightly above ground

            # Orientation based on direction
            direction_2d = barrier.ground_projection['direction'][:2]
            if np.linalg.norm(direction_2d) > 0:
                angle = np.arctan2(direction_2d[1], direction_2d[0])
                quat = tft.quaternion_from_euler(0, 0, angle)
                area_marker.pose.orientation.x = quat[0]
                area_marker.pose.orientation.y = quat[1]
                area_marker.pose.orientation.z = quat[2]
                area_marker.pose.orientation.w = quat[3]

            # Size based on line length and margin
            area_marker.scale.x = float(barrier.ground_projection['length'])
            area_marker.scale.y = float(self.ground_margin * 2)  # Width of the area
            area_marker.scale.z = 0.02  # Thin rectangle

            # Semi-transparent red
            area_marker.color.r, area_marker.color.g, area_marker.color.b, area_marker.color.a = (1, 0, 0, 0.25)

            markers.append(area_marker)

        return markers

    def _associate_detections(self, detections):
        """Associate current detections with tracked barriers"""
        matched_pairs = []
        unmatched_detections = list(range(len(detections)))
        unmatched_tracks = list(range(len(self.tracked_barriers)))

        # Simple greedy association (good enough for most cases)
        for det_idx, detection in enumerate(detections):
            if det_idx not in unmatched_detections:
                continue

            best_match = None
            best_track_idx = None

            for track_idx in unmatched_tracks:
                barrier = self.tracked_barriers[track_idx]

                if barrier.is_similar(detection['center'], detection['direction'],
                                      self.association_distance, self.association_angle):
                    best_match = barrier
                    best_track_idx = track_idx
                    break  # Take first match (greedy)

            if best_match is not None:
                matched_pairs.append((det_idx, best_track_idx))
                unmatched_detections.remove(det_idx)
                unmatched_tracks.remove(best_track_idx)

        return matched_pairs, unmatched_detections, unmatched_tracks

    def _depth_cb(self, msg: Image):
        """Main depth processing pipeline with tracking (controlled by scan frequency)"""
        if self.intrinsic is None:
            return

        # Convert depth image to numpy array
        depth_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough").copy()

        # Apply distance filtering
        if msg.encoding == "32FC1":  # meters
            depth_np[(depth_np < self.range_min) | (depth_np > self.range_max)] = 0.0
        else:  # millimeters
            rmin_mm = int(self.range_min * 1000)
            rmax_mm = int(self.range_max * 1000)
            depth_np[(depth_np < rmin_mm) | (depth_np > rmax_mm)] = 0

        # Create point cloud from depth
        depth_o3 = o3d.geometry.Image(depth_np.astype(np.float32))
        pcd = o3d.geometry.PointCloud.create_from_depth_image(
            depth_o3, self.intrinsic,
            depth_scale=1.0 if msg.encoding == "32FC1" else 1000.0,
            depth_trunc=self.range_max,
            stride=1,
        )

        # Process detections
        current_detections = []

        if len(pcd.points) > 0:
            # Downsample point cloud
            pcd = pcd.voxel_down_sample(self.voxel_size)

            # Transform to target frame
            pts_cam = np.asarray(pcd.points)
            if pts_cam.shape[0] >= self.db_min_pts:
                try:
                    tf_msg = self.tf_buffer.lookup_transform(
                        self.target_frame, self.camera_frame,
                        msg.header.stamp, rospy.Duration(1.0))

                    T = self._tfmsg_to_matrix(tf_msg)
                    pts_map = (T[:3, :3] @ pts_cam.T).T + T[:3, 3]

                    # Filter by actual height in target frame
                    height_mask = (pts_map[:, 2] >= self.height_min) & (pts_map[:, 2] <= self.height_max)
                    pts_map = pts_map[height_mask]

                    if pts_map.shape[0] >= self.db_min_pts:
                        # Create new point cloud in target frame
                        pcd = o3d.geometry.PointCloud()
                        pcd.points = o3d.utility.Vector3dVector(pts_map)

                        # DBSCAN clustering
                        labels = np.array(
                            pcd.cluster_dbscan(eps=self.db_eps, min_points=self.db_min_pts, print_progress=False))
                        n_clusters = labels.max() + 1

                        # Process each cluster
                        for cid in range(n_clusters):
                            idx = np.where(labels == cid)[0]
                            cluster_pts = np.asarray(pcd.points)[idx]

                            if cluster_pts.shape[0] < 3:
                                continue

                            # Check if cluster is rope-like using PCA
                            pca = PCA(n_components=3).fit(cluster_pts)
                            eig = pca.explained_variance_
                            if eig[0] / eig[1] < self.eigen_ratio:
                                continue

                            # Validate rope radius
                            radius = estimate_radius(eig)
                            if not (self.rope_r_min <= radius <= self.rope_r_max):
                                continue

                            # Extract features
                            center, direction, length = self._extract_line_features(cluster_pts)

                            # Sort points along principal axis for visualization
                            axis = pca.components_[0]
                            ordered_pts = cluster_pts[np.argsort(cluster_pts @ axis)]

                            current_detections.append({
                                'center': center,
                                'direction': direction,
                                'length': length,
                                'points': ordered_pts,
                                'raw_points': cluster_pts
                            })

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn_throttle(5.0, f"TF lookup failed: {e}")

        # Update tracking only when called by timer or when scan_frequency <= 0
        if self.scan_frequency <= 0:
            self._update_tracking(current_detections)
            self._publish_markers(msg.header.stamp)
        else:
            # Just update detections, timer will handle the rest
            self._update_detections_only(current_detections)

    def _update_detections_only(self, detections):
        """Update only the detection matching without publishing (used by timer-controlled mode)"""
        # Associate detections with existing tracks
        matched_pairs, unmatched_detections, unmatched_tracks = self._associate_detections(detections)

        # Update matched tracks
        for det_idx, track_idx in matched_pairs:
            detection = detections[det_idx]
            barrier = self.tracked_barriers[track_idx]
            barrier.update(detection['center'], detection['direction'],
                           detection['length'], detection['points'])

        # Create new tracks for unmatched detections
        for det_idx in unmatched_detections:
            detection = detections[det_idx]
            new_barrier = TrackedBarrier(
                detection['center'], detection['direction'],
                detection['length'], detection['points']
            )
            self.tracked_barriers.append(new_barrier)

    def _update_barrier_status(self):
        """Update barrier status (called by timer)"""
        # Remove old tracks and clean up their VO IDs
        barriers_to_remove = []
        for barrier in self.tracked_barriers:
            if barrier.should_delete(self.max_age, self.min_confidence_to_exist):
                barriers_to_remove.append(barrier)

        old_count = len(self.tracked_barriers)
        if barriers_to_remove:
            # Clean up VO IDs for deleted barriers
            for barrier in barriers_to_remove:
                if barrier.stable_id is not None:
                    self._cleanup_vo_ids_for_barrier(barrier.stable_id)

            # Remove barriers
            self.tracked_barriers = [b for b in self.tracked_barriers if b not in barriers_to_remove]
            rospy.loginfo(f"Removed {old_count - len(self.tracked_barriers)} expired barriers")

        # Assign stable IDs and manage publishing status
        for barrier in self.tracked_barriers:
            # Assign stable ID when barrier should start being published
            if barrier.should_start_publishing(self.min_consecutive_detections, self.min_confidence_to_publish):
                if barrier.stable_id is None:
                    barrier.stable_id = self.next_stable_id
                    self.next_stable_id += 1
                    rospy.loginfo(f"New stable barrier confirmed with ID {barrier.stable_id} "
                                  f"(detections: {barrier.consecutive_detections}, confidence: {barrier.confidence:.2f})")
                barrier.is_published = True

            # Check if should stop publishing
            elif barrier.is_published and barrier.should_stop_publishing(self.max_consecutive_misses,
                                                                         self.min_confidence_to_keep):
                barrier.is_published = False
                rospy.loginfo(f"Barrier {barrier.stable_id} unpublished "
                              f"(misses: {barrier.consecutive_misses}, confidence: {barrier.confidence:.2f})")

    def _update_tracking(self, detections):
        """Update tracking state with current detections (legacy method for non-timer mode)"""
        # Associate detections with existing tracks
        matched_pairs, unmatched_detections, unmatched_tracks = self._associate_detections(detections)

        # Update matched tracks
        for det_idx, track_idx in matched_pairs:
            detection = detections[det_idx]
            barrier = self.tracked_barriers[track_idx]
            barrier.update(detection['center'], detection['direction'],
                           detection['length'], detection['points'])

        # Decay confidence of unmatched tracks
        for track_idx in unmatched_tracks:
            self.tracked_barriers[track_idx].decay()

        # Create new tracks for unmatched detections
        for det_idx in unmatched_detections:
            detection = detections[det_idx]
            new_barrier = TrackedBarrier(
                detection['center'], detection['direction'],
                detection['length'], detection['points']
            )
            self.tracked_barriers.append(new_barrier)

        # Update barrier status
        self._update_barrier_status()

    def _publish_markers(self, stamp):
        """Publish markers for barriers that should be published"""
        marray = MarkerArray()
        current_marker_ids = set()

        # Add markers for barriers that should be published
        for barrier in self.tracked_barriers:
            if barrier.is_published and barrier.stable_id is not None:
                # 3D barrier line marker
                barrier_marker = Marker()
                barrier_marker.header.stamp = stamp
                barrier_marker.header.frame_id = self.target_frame
                barrier_marker.ns = "stable_barriers"
                barrier_marker.id = barrier.stable_id
                barrier_marker.type = Marker.LINE_STRIP
                barrier_marker.action = Marker.ADD
                barrier_marker.scale.x = self.marker_w
                barrier_marker.lifetime = rospy.Duration(0)  # Persistent markers

                # 改进的颜色编码：基于历史置信度和当前状态
                if barrier.consecutive_misses == 0:
                    # Recently detected - bright green
                    barrier_marker.color.r, barrier_marker.color.g, barrier_marker.color.b, barrier_marker.color.a = (0,
                                                                                                                      1,
                                                                                                                      0,
                                                                                                                      1)
                elif barrier.consecutive_misses < 10:
                    # Recently seen - yellow
                    barrier_marker.color.r, barrier_marker.color.g, barrier_marker.color.b, barrier_marker.color.a = (1,
                                                                                                                      1,
                                                                                                                      0,
                                                                                                                      1)
                elif barrier.peak_confidence > 0.8:
                    # High confidence memory - blue
                    barrier_marker.color.r, barrier_marker.color.g, barrier_marker.color.b, barrier_marker.color.a = (0,
                                                                                                                      0,
                                                                                                                      1,
                                                                                                                      0.8)
                else:
                    # Not seen for a while - orange
                    barrier_marker.color.r, barrier_marker.color.g, barrier_marker.color.b, barrier_marker.color.a = (1,
                                                                                                                      0.5,
                                                                                                                      0,
                                                                                                                      0.7)

                barrier_marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                                         for p in barrier.points]
                marray.markers.append(barrier_marker)
                current_marker_ids.add(barrier.stable_id)

                # Ground projection markers
                ground_markers = self._create_ground_projection_markers(barrier, stamp)
                marray.markers.extend(ground_markers)

        # Delete old markers
        for old_id in self.published_marker_ids - current_marker_ids:
            # Delete 3D barrier
            delete_marker = Marker()
            delete_marker.header.stamp = stamp
            delete_marker.header.frame_id = self.target_frame
            delete_marker.ns = "stable_barriers"
            delete_marker.id = old_id
            delete_marker.action = Marker.DELETE
            marray.markers.append(delete_marker)

            # Delete ground projection
            delete_ground = Marker()
            delete_ground.header.stamp = stamp
            delete_ground.header.frame_id = self.target_frame
            delete_ground.ns = "ground_projections"
            delete_ground.id = old_id
            delete_ground.action = Marker.DELETE
            marray.markers.append(delete_ground)

            # Delete ground area
            delete_area = Marker()
            delete_area.header.stamp = stamp
            delete_area.header.frame_id = self.target_frame
            delete_area.ns = "ground_areas"
            delete_area.id = old_id
            delete_area.action = Marker.DELETE
            marray.markers.append(delete_area)

        # Update published marker IDs
        self.published_marker_ids = current_marker_ids

        # Publish markers
        self.marker_pub.publish(marray)

    @staticmethod
    def _tfmsg_to_matrix(tf_msg) -> np.ndarray:
        """Convert TF message to 4x4 transformation matrix"""
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        M = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        M[0, 3], M[1, 3], M[2, 3] = t.x, t.y, t.z
        return M


if __name__ == "__main__":
    rospy.init_node("depth_barrier_detector", anonymous=False)
    DepthBarrierDetector()
    rospy.loginfo("Optimized depth barrier detector for navigation safety started.")
    rospy.spin()