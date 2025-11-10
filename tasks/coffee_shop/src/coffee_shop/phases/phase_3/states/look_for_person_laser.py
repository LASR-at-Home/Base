import smach
import rospy
from sensor_msgs.msg import LaserScan, CameraInfo, Image
from geometry_msgs.msg import Point, PointStamped
import numpy as np
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
from image_geometry import PinholeCameraModel
from play_motion_msgs.msg import PlayMotionGoal
import time
from shapely.geometry import Point as ShapelyPoint, Polygon


def timeit_rospy(method):
    """Decorator for timing ROS methods"""

    def timed(*args, **kw):
        ts = time.time()
        result = method(*args, **kw)
        te = time.time()
        rospy.loginfo("%r  %2.2f s" % (method.__name__, (te - ts)))
        return result

    return timed


class LookForPersonLaser(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=["found", "not found"])
        self.context = context
        self.camera = PinholeCameraModel()
        self.camera.fromCameraInfo(
            rospy.wait_for_message("/xtion/rgb/camera_info", CameraInfo)
        )
        self.corners = rospy.get_param("/coffee_shop/wait/cuboid")

    @timeit_rospy
    def get_points_and_pixels_from_laser(self, msg):
        """Converts a LaserScan message to a collection of points in camera frame, with their corresponding pixel values in a flat array. The method pads out the points to add vertical "pillars" to the point cloud.

        Args:
            msg (LaserScan): ROS Laser Scan message from /scan topic.
        Returns:
            List: A list of tuples containing all the filtered and padded points in camera frame.
            List: A list of pixel values corresponding to the points in the first list.
        """
        # First get the laser scan points, and then convert to camera frame
        pcl_msg = lg.LaserProjection().projectLaser(msg)
        pcl_points = [
            p for p in  # pc2.read_points(pcl_msg, field_names=("x, y, z"), skip_nans=True)
            pc2.read_points(pcl_msg, field_names=("x", "y", "z"), skip_nans=True)

        ]

        padded_points = []
        pixels = []
        for point in pcl_points:
            # Pad out the points to add vertical "pillars" to the point cloud
            for z in np.linspace(0.0, 1.0, 5):
                padded_points.append(
                    PointStamped(
                        point=Point(x=point[0], y=point[1], z=z), header=msg.header
                    )
                )

        # tf_points = self.context.tf_point_list(padded_points, "base_laser_link", "xtion_rgb_optical_frame")

        tf_points = self.context.tf_point_list(
            padded_points, msg.header.frame_id, "xtion_rgb_optical_frame"
        )

        padded_converted_points = []
        for p in tf_points:
            pt = (p.point.x, p.point.y, p.point.z)
            u, v = self.camera.project3dToPixel(pt)
            # Filter out points that are outside the camera frame
            if u >= 0 and u < 640 and v >= 0 and v < 480:
                pixels.append(u)
                pixels.append(v)
                padded_converted_points.append(p)

        return padded_converted_points, pixels

    @timeit_rospy
    def convert_points_to_map_frame(self, points, from_frame="xtion_rgb_optical_frame"):
        """Converts a list of points in camera frame to a list of points in map frame.

        Args:
            points (List): A list of tuples containing points in camera frame.
        Returns:
            List: A list of tuples containing points in map frame.
        """
        tf_points = self.context.tf_point_list(points, from_frame, "map")
        converted_points = [(p.point.x, p.point.y, p.point.z) for p in tf_points]

        return converted_points

    def execute(self, userdata):
        self.context.stop_head_manager("head_manager")
        pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)

        lsr_scan = rospy.wait_for_message("/scan", LaserScan)
        img_msg = rospy.wait_for_message("/xtion/rgb/image_raw", Image)

        points, pixels = self.get_points_and_pixels_from_laser(lsr_scan)
        detections = self.context.yolo(img_msg, self.context.YOLO_person_model, 0.3, [])
        # waiting_area = Polygon(self.corners)
        pixels_2d = np.array(pixels).reshape(-1, 2)

        for detection in detections.detected_objects:
            if detection.name != "person":
                continue

            # --- scale detection polygon to raw image size ---
            W, H = img_msg.width, img_msg.height
            poly_pix = np.array(detection.xyseg, dtype=float).reshape(-1, 2)

            # if normalized [0,1], scale to pixels
            if poly_pix.max() <= 1.5:
                poly_pix[:, 0] *= W
                poly_pix[:, 1] *= H
            else:
                # if YOLO ran on a resized image, map to the raw image size
                det_w = getattr(detection, "img_width", W)
                det_h = getattr(detection, "img_height", H)
                sx, sy = float(W) / det_w, float(H) / det_h
                poly_pix[:, 0] *= sx
                poly_pix[:, 1] *= sy

            # person polygon in image space (softened edges)
            person_poly = Polygon(poly_pix).buffer(0.10)

            # projected laser pixels that fall inside the person polygon
            satisfied = [person_poly.covers(ShapelyPoint(x, y)) for x, y in pixels_2d]
            idx_hits = [i for i, ok in enumerate(satisfied) if ok]
            if not idx_hits:
                continue

            # map-frame points for those hits
            filtered_points = self.convert_points_to_map_frame([points[i] for i in idx_hits])

            if self.corners is not None:
                # waiting area polygon in MAP frame (softened, boundary-safe)
                wait_poly = Polygon(self.corners).buffer(0.10)
                in_area = [wait_poly.covers(ShapelyPoint(px, py)) for (px, py, pz) in filtered_points]
                idx_area = [i for i, ok in enumerate(in_area) if ok]
                if not idx_area:
                    continue
                points_inside_area = [filtered_points[i] for i in idx_area]
                point = np.mean(points_inside_area, axis=0)
            else:
                point = np.mean(filtered_points, axis=0)

            # publish/store target and finish
            self.context.publish_person_pose(*point, "map")
            self.context.new_customer_pose = PointStamped(point=Point(*point))
            self.context.new_customer_pose.header.frame_id = "map"
            return "found"

        self.context.start_head_manager("head_manager", "")

        return "not found"
