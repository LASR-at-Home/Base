#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

import open3d as o3d
from sklearn.decomposition import PCA
import tf2_ros
import tf.transformations as tft


def estimate_radius(eigen_vals: np.ndarray) -> float:
    """Estimate rope radius from PCA eigenvalues"""
    return float(np.sqrt(eigen_vals[1]))


class DepthBarrierDetector:
    def __init__(self):
        # ROS parameters
        self.height_min = rospy.get_param("~height_min", 0.30)
        self.height_max = rospy.get_param("~height_max", 2.50)
        self.rope_r_min = rospy.get_param("~rope_radius_min", 0.001)
        self.rope_r_max = rospy.get_param("~rope_radius_max", 0.15)
        self.voxel_size = rospy.get_param("~voxel_size", 0.01)
        self.db_eps = rospy.get_param("~db_eps", 0.03)
        self.db_min_pts = rospy.get_param("~db_min_pts", 30)
        self.eigen_ratio = rospy.get_param("~eigen_ratio", 10.0)
        self.marker_w = rospy.get_param("~marker_width", 0.01)
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.range_min = rospy.get_param("~range_min", 0.30)
        self.range_max = rospy.get_param("~range_max", 5.00)
        self.marker_lifetime = rospy.get_param("~marker_lifetime", 0.5)

        # ROS communication
        self.bridge = CvBridge()
        self.intrinsic = None
        self.camera_frame = None

        # TF components
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Marker management for ghost trail prevention
        self.marker_id_counter = 0
        self.active_marker_ids = set()

        # Publishers
        self.marker_pub = rospy.Publisher("barrier_marker_array", MarkerArray, queue_size=1)

        # Subscribers
        rospy.Subscriber("/xtion/depth_registered/camera_info", CameraInfo, self._cam_info_cb, queue_size=1)
        rospy.Subscriber("/xtion/depth_registered/image_raw", Image, self._depth_cb, queue_size=1, buff_size=2**22)

    def _cam_info_cb(self, msg: CameraInfo):
        """Initialize camera intrinsics once"""
        if self.intrinsic is None:
            self.intrinsic = o3d.camera.PinholeCameraIntrinsic(
                msg.width, msg.height,
                msg.K[0], msg.K[4], msg.K[2], msg.K[5]
            )
            self.camera_frame = msg.header.frame_id or "xtion_depth_optical_frame"
            rospy.loginfo("Camera intrinsics received: frame = %s", self.camera_frame)

    def _depth_cb(self, msg: Image):
        """Main depth processing pipeline"""
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
        if len(pcd.points) == 0:
            self._publish_empty(msg.header.stamp)
            return

        # Downsample point cloud
        pcd = pcd.voxel_down_sample(self.voxel_size)

        # Transform to target frame FIRST, then filter by actual height
        pts_cam = np.asarray(pcd.points)
        if pts_cam.shape[0] < self.db_min_pts:
            self._publish_empty(msg.header.stamp)
            return

        # Transform to target frame with proper timestamp sync
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.target_frame, self.camera_frame,
                msg.header.stamp, rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"TF lookup failed: {e}")
            return

        T = self._tfmsg_to_matrix(tf_msg)
        pts_map = (T[:3, :3] @ pts_cam.T).T + T[:3, 3]

        # Filter by ACTUAL height in target frame (assuming Z-axis is vertical)
        # In map frame, Z typically represents actual height above ground
        height_mask = (pts_map[:, 2] >= self.height_min) & (pts_map[:, 2] <= self.height_max)
        pts_map = pts_map[height_mask]

        if pts_map.shape[0] < self.db_min_pts:
            self._publish_empty(msg.header.stamp)
            return

        # Create new point cloud in target frame
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts_map)

        # DBSCAN clustering
        labels = np.array(pcd.cluster_dbscan(eps=self.db_eps, min_points=self.db_min_pts, print_progress=False))
        n_clusters = labels.max() + 1
        if n_clusters == 0:
            self._publish_empty(msg.header.stamp)
            return

        # Build marker array with ghost trail prevention
        marray = MarkerArray()
        new_marker_ids = set()

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

            # Sort points along principal axis
            axis = pca.components_[0]
            ordered_pts = cluster_pts[np.argsort(cluster_pts @ axis)]

            # Create marker with unique ID
            marker_id = self.marker_id_counter
            self.marker_id_counter += 1
            new_marker_ids.add(marker_id)

            marker = Marker()
            marker.header.stamp = msg.header.stamp
            marker.header.frame_id = self.target_frame
            marker.ns = "barrier_lines"
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = self.marker_w
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1, 0, 0, 1)
            marker.lifetime = rospy.Duration(self.marker_lifetime)
            marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in ordered_pts]
            marray.markers.append(marker)

        # Remove old markers that are no longer detected
        for old_id in self.active_marker_ids - new_marker_ids:
            delete_marker = Marker()
            delete_marker.header.stamp = msg.header.stamp
            delete_marker.header.frame_id = self.target_frame
            delete_marker.ns = "barrier_lines"
            delete_marker.id = old_id
            delete_marker.action = Marker.DELETE
            marray.markers.append(delete_marker)

        # Update active marker set
        self.active_marker_ids = new_marker_ids

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

    def _publish_empty(self, stamp):
        """Clear all markers"""
        ma = MarkerArray()
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self.target_frame
        m.action = Marker.DELETEALL
        ma.markers.append(m)
        self.marker_pub.publish(ma)
        self.active_marker_ids.clear()


if __name__ == "__main__":
    rospy.init_node("depth_barrier_detector", anonymous=False)
    DepthBarrierDetector()
    rospy.loginfo("Depth barrier detector node started.")
    rospy.spin()
