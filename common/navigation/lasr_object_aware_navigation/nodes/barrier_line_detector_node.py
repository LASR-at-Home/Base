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
    return float(np.sqrt(eigen_vals[1]))


class DepthBarrierDetector:
    def __init__(self):
        # ---------------- ROS parameters ---------------- #
        self.height_min  = rospy.get_param("~height_min",        0.20)
        self.height_max  = rospy.get_param("~height_max",        2.50)
        self.rope_r_min  = rospy.get_param("~rope_radius_min",   0.001)
        self.rope_r_max  = rospy.get_param("~rope_radius_max",   0.5)
        self.voxel_size  = rospy.get_param("~voxel_size",        0.01)
        self.db_eps      = rospy.get_param("~db_eps",            0.03)
        self.db_min_pts  = rospy.get_param("~db_min_pts",        30)
        self.eigen_ratio = rospy.get_param("~eigen_ratio",       10.0)
        self.marker_w    = rospy.get_param("~marker_width",      0.01)
        self.target_frame = rospy.get_param("~target_frame",     "map")
        self.range_min = rospy.get_param("~range_min", 0.30)
        self.range_max = rospy.get_param("~range_max", 5.00)
        # ------------------------------------------------- #

        self.bridge = CvBridge()
        self.intrinsic = None
        self.camera_frame = None

        # tf2 buffer & listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publisher
        self.marker_pub = rospy.Publisher("barrier_marker_array",
                                          MarkerArray, queue_size=1)

        # Subscribers
        rospy.Subscriber("/xtion/depth_registered/camera_info",
                         CameraInfo, self._cam_info_cb, queue_size=1)
        rospy.Subscriber("/xtion/depth_registered/image_raw",
                         Image, self._depth_cb,   queue_size=1,
                         buff_size=2 ** 22)  # reduce latency on large depth images

    # -------------- CameraInfo callback -------------- #
    def _cam_info_cb(self, msg: CameraInfo):
        if self.intrinsic is None:
            self.intrinsic = o3d.camera.PinholeCameraIntrinsic(
                msg.width, msg.height,
                msg.K[0], msg.K[4], msg.K[2], msg.K[5]
            )
            self.camera_frame = msg.header.frame_id or "xtion_depth_optical_frame"
            rospy.loginfo("Camera intrinsics received: frame = %s", self.camera_frame)

    # -------------- Depth callback -------------- #
    def _depth_cb(self, msg: Image):
        if self.intrinsic is None:
            return  # wait for intrinsics first

        depth_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Make the array writable (cv_bridge returns a read-only view)
        depth_np = depth_np.copy()

        # ---- distance filter on raw depth ----
        if msg.encoding == "32FC1":  # depth in meters
            depth_np[(depth_np < self.range_min) |
                     (depth_np > self.range_max)] = 0.0
        else:  # depth in millimeters
            rmin_mm = int(self.range_min * 1000)
            rmax_mm = int(self.range_max * 1000)
            depth_np[(depth_np < rmin_mm) |
                     (depth_np > rmax_mm)] = 0
        # --------------------------------------

        depth_o3 = o3d.geometry.Image(depth_np.astype(np.float32))
        pcd = o3d.geometry.PointCloud.create_from_depth_image(
            depth_o3, self.intrinsic,
            depth_scale=1.0 if msg.encoding == "32FC1" else 1000.0,
            depth_trunc=self.range_max,  # OPTIONAL: stop Open3D at same max
            stride=1,
        )
        if len(pcd.points) == 0:
            return

        pcd = pcd.voxel_down_sample(self.voxel_size)

        # ----------------- Height filter (camera frame) ----------------- #
        pts_cam = np.asarray(pcd.points)
        mask = (pts_cam[:, 2] >= self.height_min) & (pts_cam[:, 2] <= self.height_max)
        pts_cam = pts_cam[mask]
        if pts_cam.shape[0] < self.db_min_pts:
            self._publish_empty(msg.header.stamp)
            return

        # ----------------- Transform to target (map) frame ---------------- #
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.target_frame, self.camera_frame,
                rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logwarn_throttle(5.0, "TF lookup failed.")
            return

        T = self._tfmsg_to_matrix(tf_msg)
        pts_map = (T[:3, :3] @ pts_cam.T).T + T[:3, 3]

        # Replace point cloud data with transformed points for clustering
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts_map)

        # ----------------- DBSCAN clustering (map frame) ----------------- #
        labels = np.array(pcd.cluster_dbscan(
            eps=self.db_eps, min_points=self.db_min_pts, print_progress=False))
        n_clusters = labels.max() + 1
        if n_clusters == 0:
            self._publish_empty(msg.header.stamp)
            return

        # ----------------- Build MarkerArray ----------------- #
        marray = MarkerArray()
        for cid in range(n_clusters):
            idx = np.where(labels == cid)[0]
            cluster_pts = np.asarray(pcd.points)[idx]

            if cluster_pts.shape[0] < 3:
                continue

            # PCA elongation check
            pca = PCA(n_components=3).fit(cluster_pts)
            eig = pca.explained_variance_
            if eig[0] / eig[1] < self.eigen_ratio:
                continue

            # Radius constraint
            radius = estimate_radius(eig)
            if not (self.rope_r_min <= radius <= self.rope_r_max):
                continue

            # Sort points along principal axis
            axis = pca.components_[0]
            ordered_pts = cluster_pts[np.argsort(cluster_pts @ axis)]

            # LINE_STRIP marker in target frame
            marker = Marker()
            marker.header.stamp = msg.header.stamp
            marker.header.frame_id = self.target_frame
            marker.ns  = "barrier_lines"
            marker.id  = cid
            marker.type   = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = self.marker_w
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1, 0, 0, 1)
            marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                             for p in ordered_pts]
            marray.markers.append(marker)

        # Publish markers
        if marray.markers:
            self.marker_pub.publish(marray)
        else:
            self._publish_empty(msg.header.stamp)

    # ----------------- Utility: TF → 4×4 matrix ----------------- #
    @staticmethod
    def _tfmsg_to_matrix(tf_msg) -> np.ndarray:
        """Convert geometry_msgs/TransformStamped to 4×4 homogeneous matrix."""
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        M = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        M[0, 3], M[1, 3], M[2, 3] = t.x, t.y, t.z
        return M

    # ----------------- Utility: clear markers ----------------- #
    def _publish_empty(self, stamp):
        ma = MarkerArray()
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self.target_frame
        m.action = Marker.DELETEALL
        ma.markers.append(m)
        self.marker_pub.publish(ma)


# ------------------------------- Main ------------------------------- #
if __name__ == "__main__":
    rospy.init_node("depth_barrier_detector", anonymous=False)
    DepthBarrierDetector()
    rospy.loginfo("Depth barrier detector node started.")
    rospy.spin()