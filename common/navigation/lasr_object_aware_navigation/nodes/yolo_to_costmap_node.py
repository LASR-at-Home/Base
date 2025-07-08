#!/usr/bin/env python3
import rospy
import numpy as np
import math
import collections

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, LaserScan
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PointStamped, Point

from lasr_vision_msgs.srv import YoloDetection3D, YoloDetection3DRequest

import tf2_ros
import tf2_geometry_msgs
import cv2

COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush"
]

class YoloToCostmapNode:
    def __init__(self):
        # Basic parameters for costmap and detection
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.grid_resolution = rospy.get_param("~resolution", 0.05)
        self.grid_width = rospy.get_param("~width", 200)
        self.grid_height = rospy.get_param("~height", 200)
        self.obstacle_radius = rospy.get_param("~obstacle_radius", 0.75)
        self.yolo_service = rospy.get_param("~yolo_service", "/yolo/detect3d")
        self.yolo_model = rospy.get_param("~yolo_model", "yolo11l-seg.pt")
        self.confidence = rospy.get_param("~confidence", 0.15)
        self.class_filter = rospy.get_param("~filter", COCO_CLASSES)
        self.detection_interval = rospy.get_param("~interval", 1.0)

        # New: buffer and publish rates
        self.scan_buffer_size = rospy.get_param("~scan_buffer_size", 30)  # frames for historical filter
        self.scan_publish_rate = rospy.get_param("~scan_publish_rate", 5.0)  # Hz

        # LaserScan simulation params (now parameterized)
        self.scan_angle_min = rospy.get_param("~scan_angle_min", -math.pi/3)
        self.scan_angle_max = rospy.get_param("~scan_angle_max", math.pi/3)
        self.scan_angle_inc = rospy.get_param("~scan_angle_inc", math.radians(1.0))
        self.scan_range_min = rospy.get_param("~scan_range_min", 0.05)
        self.scan_range_max = rospy.get_param("~scan_range_max", 10.0)

        # Advanced filter
        self.ignore_labels = set([label.lower() for label in rospy.get_param("~ignore_labels", ["person"])])
        self.max_height = rospy.get_param("~max_height", 0.2)  # meters, 0 disables

        # Image/depth/camera topics
        self.image_topic = rospy.get_param("~image_topic", "/xtion/rgb/image_raw")
        self.depth_topic = rospy.get_param("~depth_image_topic", "/xtion/depth_registered/image_raw")
        self.camera_info_topic = rospy.get_param("~depth_camera_info_topic", "/xtion/depth_registered/camera_info")

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_depth = None
        self.latest_info = None

        rospy.Subscriber(self.image_topic, Image, self.cb_image, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self.cb_depth, queue_size=1)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.cb_info, queue_size=1)

        # Costmap publisher (OccupancyGrid)
        self.costmap_pub = rospy.Publisher("/custom_obstacle_map", OccupancyGrid, queue_size=1, latch=True)
        # PointCloud2 publisher for /_rgbd_scan (as before)
        self.pointcloud_pub = rospy.Publisher("/_rgbd_scan", PointCloud2, queue_size=1)
        # Additional LaserScan publisher for simulated scan
        self.laserscan_pub = rospy.Publisher("/rgbd_scan", LaserScan, queue_size=1)

        # TF listener for frame conversion
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("Waiting for YOLO service: %s...", self.yolo_service)
        rospy.wait_for_service(self.yolo_service)
        self.yolo_srv = rospy.ServiceProxy(self.yolo_service, YoloDetection3D)
        rospy.loginfo("YOLO 3D service ready.")

        # History buffer for obstacle points, capacity = scan_buffer_size
        self.obstacle_point_buffer = collections.deque(maxlen=self.scan_buffer_size)

        # Main detection timer (YOLO + grid update)
        self.timer = rospy.Timer(rospy.Duration(self.detection_interval), self.timer_cb)
        # High-rate scan/pointcloud publisher timer
        self.scan_timer = rospy.Timer(rospy.Duration(1.0/self.scan_publish_rate), self.scan_publish_cb)

        self.latest_obstacle_points = []

    def cb_image(self, msg):
        self.latest_image = msg

    def cb_depth(self, msg):
        self.latest_depth = msg

    def cb_info(self, msg):
        self.latest_info = msg

    def timer_cb(self, event):
        if self.latest_image is None or self.latest_depth is None or self.latest_info is None:
            rospy.logwarn_throttle(10.0, "Waiting for image/depth/camera_info...")
            return
        try:
            detections = self.call_yolo_service()
            self.publish_obstacle_grid(detections)
        except Exception as e:
            rospy.logwarn("Failed to get YOLO detections or publish grid: %s", str(e))

    def call_yolo_service(self):
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

    def publish_obstacle_grid(self, detections):
        grid = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)
        ox = self.grid_width * self.grid_resolution / 2
        oy = self.grid_height * self.grid_resolution / 2

        # Camera intrinsics for mask processing (not used for scan)
        K = self.latest_info.K
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]

        # Try to get depth image as numpy array
        try:
            depth_cv = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding="passthrough")
        except Exception as e:
            rospy.logwarn("Failed to decode depth image: %s", str(e))
            return

        count = 0
        obstacle_points = []
        for idx, det in enumerate(detections):
            label = getattr(det, "name", "").lower()
            info_msg = f"[{idx}] Class: '{label}' "

            # Filter out ignored labels
            if label in self.ignore_labels:
                rospy.loginfo(info_msg + "Skipped: in ignore_labels.")
                continue

            # Mask-based median height calculation
            real_height = None
            if hasattr(det, "xyseg") and det.xyseg and len(det.xyseg) >= 6:
                try:
                    contour = np.array(det.xyseg, dtype=np.int32).reshape(-1, 2)
                    mask = np.zeros(depth_cv.shape[:2], dtype=np.uint8)
                    cv2.fillPoly(mask, [contour], color=255)
                    indices = np.argwhere(mask == 255)
                    if indices.shape[0] == 0:
                        rospy.loginfo(info_msg + "Mask valid pixel count = 0. Skipping.")
                        continue

                    us = indices[:, 1]
                    vs = indices[:, 0]
                    zs = depth_cv[vs, us]
                    valid = (zs > 0) & (~np.isnan(zs))
                    if not np.any(valid):
                        rospy.loginfo(info_msg + "No valid depth points in mask. Skipping.")
                        continue

                    valid_us = us[valid]
                    valid_vs = vs[valid]
                    valid_zs = zs[valid]
                    median_idx = np.argsort(valid_zs)[len(valid_zs) // 2]
                    u_med = valid_us[median_idx]
                    v_med = valid_vs[median_idx]
                    z_med = valid_zs[median_idx]
                    x_cam = (u_med - cx) * z_med / fx
                    y_cam = (v_med - cy) * z_med / fy

                    pt_cam = PointStamped()
                    pt_cam.header = self.latest_depth.header
                    pt_cam.point = Point(x_cam, y_cam, z_med)
                    try:
                        pt_map = self.tf_buffer.transform(pt_cam, self.target_frame, rospy.Duration(0.25))
                        real_height = abs(pt_map.point.z)
                        rospy.loginfo(info_msg + f"Mask median point height: {real_height:.3f} m (z in {self.target_frame})")
                    except Exception as ex:
                        rospy.logwarn(info_msg + f"TF transform failed: {ex}")
                        continue
                except Exception as ex:
                    rospy.logwarn(info_msg + f"Error extracting world height: {ex}")
                    continue
            else:
                rospy.loginfo(info_msg + "No mask (xyseg) available, skipping height check.")

            # Filter out by real height
            if self.max_height > 0 and real_height is not None:
                if real_height > self.max_height:
                    rospy.loginfo(info_msg + f"Skipped: mask median height {real_height:.3f} > max_height={self.max_height:.2f}")
                    continue

            # Draw obstacle on occupancy grid (center in map frame)
            if not hasattr(det, "point") or det.point is None:
                rospy.logwarn(info_msg + "-- Skipped: No valid 3D point.")
                continue
            x, y = det.point.x, det.point.y
            z = det.point.z
            if 0 < z < self.max_height:
                obstacle_points.append((x, y, z))

            r = self.obstacle_radius
            cx_grid = int((x + ox) / self.grid_resolution)
            cy_grid = int((y + oy) / self.grid_resolution)
            gr = int(r / self.grid_resolution)
            yy, xx = np.ogrid[:self.grid_height, :self.grid_width]
            circle_mask = (xx - cx_grid) ** 2 + (yy - cy_grid) ** 2 <= gr ** 2
            grid[circle_mask] = 100
            count += 1
            rospy.loginfo(info_msg + f"at ({x:.2f}, {y:.2f}) -- Added as obstacle.")

        rospy.loginfo("[Costmap update] Total: %d, Added: %d", len(detections), count)
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.target_frame
        msg.info.resolution = self.grid_resolution
        msg.info.width = self.grid_width
        msg.info.height = self.grid_height
        msg.info.origin = Pose()
        msg.data = grid.reshape(-1).tolist()
        self.costmap_pub.publish(msg)
        rospy.loginfo_throttle(2.0, "Published %d virtual obstacles to /custom_obstacle_map", count)

        # Add current frame's obstacle_points to buffer (for scan/pointcloud publishing)
        self.latest_obstacle_points = obstacle_points
        if obstacle_points:
            self.obstacle_point_buffer.append(obstacle_points)
        else:
            # Still add empty to keep buffer update rate constant
            self.obstacle_point_buffer.append([])

    def scan_publish_cb(self, event):
        # Publish at high rate using latest N frames in buffer (for smooth and dense signal)
        # Combine all points from buffer
        all_points = [pt for frame in self.obstacle_point_buffer for pt in frame]
        if not all_points:
            rospy.loginfo_throttle(2.0, "[scan_publish_cb] No obstacle points to publish.")
            return

        # ======= Publish PointCloud2 =======
        pc2_msg = pc2.create_cloud_xyz32(
            header=rospy.Header(
                stamp=rospy.Time.now(),
                frame_id=self.target_frame
            ),
            points=all_points
        )
        self.pointcloud_pub.publish(pc2_msg)
        rospy.loginfo_throttle(2.0, f"Published {len(all_points)} points to /_rgbd_scan (filtered)")

        # ======= Publish LaserScan (simulated from all buffer points) =======
        n_scan = int((self.scan_angle_max - self.scan_angle_min) / self.scan_angle_inc) + 1
        scan_ranges = [float('inf')] * n_scan

        for x, y, z in all_points:
            angle = math.atan2(y, x)
            dist = math.hypot(x, y)
            idx = int((angle - self.scan_angle_min) / self.scan_angle_inc)
            if 0 <= idx < n_scan and dist > self.scan_range_min:
                if scan_ranges[idx] > dist:
                    scan_ranges[idx] = dist

        scan_msg = LaserScan()
        scan_msg.header.stamp = rospy.Time.now()
        scan_msg.header.frame_id = self.target_frame
        scan_msg.angle_min = self.scan_angle_min
        scan_msg.angle_max = self.scan_angle_max
        scan_msg.angle_increment = self.scan_angle_inc
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 1.0 / self.scan_publish_rate
        scan_msg.range_min = self.scan_range_min
        scan_msg.range_max = self.scan_range_max
        scan_msg.ranges = scan_ranges
        scan_msg.intensities = []

        self.laserscan_pub.publish(scan_msg)
        rospy.loginfo_throttle(2.0, f"Published filtered LaserScan with {len(all_points)} points to /rgbd_scan")

if __name__ == "__main__":
    rospy.init_node("yolo_to_costmap_node")
    node = YoloToCostmapNode()
    rospy.spin()
