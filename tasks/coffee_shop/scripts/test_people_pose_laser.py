#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan, CameraInfo, Image
from geometry_msgs.msg import PointStamped
from lasr_vision_msgs.srv import YoloDetection
from lasr_shapely.srv import PointsInPolygon2D
from visualization_msgs.msg import Marker
import numpy as np
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
from image_geometry import PinholeCameraModel
import tf2_ros
import tf2_geometry_msgs


def create_point_marker(x, y, z, idx):
    marker_msg = Marker()
    marker_msg.header.frame_id = "xtion_rgb_optical_frame"
    marker_msg.header.stamp = rospy.Time.now()
    marker_msg.id = idx
    marker_msg.type = Marker.SPHERE
    marker_msg.action = Marker.ADD
    marker_msg.pose.position.x = x
    marker_msg.pose.position.y = y
    marker_msg.pose.position.z = z
    marker_msg.pose.orientation.w = 1.0
    marker_msg.scale.x = 0.1
    marker_msg.scale.y = 0.1
    marker_msg.scale.z = 0.1
    marker_msg.color.a = 1.0
    marker_msg.color.r = 0.0
    marker_msg.color.g = 1.0
    marker_msg.color.b = 0.0
    return marker_msg


def get_transform(from_frame, to_frame):
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    try:
        t = tf_buffer.lookup_transform(
            to_frame, from_frame, rospy.Time(0), rospy.Duration(5)
        )
        return t
    except (
        tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException,
    ):
        raise


def apply_transform(input_xyz, transform, target="xtion_rgb_optical_frame"):
    ps = PointStamped()
    ps.point.x = input_xyz[0]
    ps.point.y = input_xyz[1]
    ps.point.z = input_xyz[2]
    ps.header.frame_id = target
    ps.header.stamp = rospy.Time.now()

    tr_point = tf2_geometry_msgs.do_transform_point(ps, transform)
    return (tr_point.point.x, tr_point.point.y, tr_point.point.z)


def get_points_and_pixels_from_laser(msg):
    """Converts a LaserScan message to a collection of points in camera frame, with their corresponding pixel values in a flat array. The method pads out the points to add vertical "pillars" to the point cloud.

    Args:
        msg (LaserScan): ROS Laser Scan message from /scan topic.
    Returns:
        List: A list of tuples containing all the filtered and padded points in camera frame.
        List: A list of pixel values corresponding to the points in the first list.
    """
    pcl_msg = lg.LaserProjection().projectLaser(msg)
    pcl_points = [
        p for p in pc2.read_points(pcl_msg, field_names=("x, y, z"), skip_nans=True)
    ]
    tr = get_transform("base_laser_link", "xtion_rgb_optical_frame")

    padded_converted_points = []
    pixels = []
    for point in pcl_points:
        for z in np.linspace(0.0, 2.0, 5):
            p = apply_transform((point[0], point[1], z), tr)
            u, v = camera.project3dToPixel(p)
            if u >= 0 and u < 640 and v >= 0 and v < 480:
                padded_converted_points.append(p)
                pixels.append(u)
                pixels.append(v)

    return padded_converted_points, pixels


if __name__ == "__main__":
    rospy.init_node("test_people_pose_laser")

    people_pose_pub = rospy.Publisher("/people_poses_laser", Marker, queue_size=100)

    rospy.wait_for_service("/yolo/detect", rospy.Duration(15.0))
    yolo = rospy.ServiceProxy("/yolo/detect", YoloDetection)
    shapely = rospy.ServiceProxy(
        "/lasr_shapely/points_in_polygon_2d", PointsInPolygon2D
    )
    camera = PinholeCameraModel()
    camera.fromCameraInfo(rospy.wait_for_message("/xtion/rgb/camera_info", CameraInfo))

    try:
        corners = rospy.get_param("/wait/cuboid")
    except KeyError:
        rospy.logwarn("No cuboid found in parameter server.")
        corners = None

    while not rospy.is_shutdown():
        lsr_scan = rospy.wait_for_message("/scan", LaserScan)
        points, pixels = get_points_and_pixels_from_laser(lsr_scan)
        img_msg = rospy.wait_for_message("/xtion/rgb/image_raw", Image)
        detections = yolo(img_msg, "yolov8n-seg.pt", 0.3, 0.3)
        i = 0
        for detection in detections.detected_objects:
            if detection.name == "person":
                decision = shapely(detection.xyseg, pixels)
                # save index for every true decision
                idx = [idx for idx, el in enumerate(decision.inside) if el]
                filtered_points = [points[i] for i in idx]
                if corners is not None:
                    waiting_area = (
                        shapely(corners, filtered_points) if corners else None
                    )
                    idx = [idx for idx, el in enumerate(waiting_area.inside) if el]
                    points_inside_area = [filtered_points[i] for i in idx]
                    if len(points_inside_area):
                        point = points_inside_area[0]
                        marker_msg = create_point_marker(
                            point[0], point[1], point[2], i
                        )
                        people_pose_pub.publish(marker_msg)
                else:
                    for point in filtered_points:
                        marker_msg = create_point_marker(
                            point[0], point[1], point[2], i
                        )
                        people_pose_pub.publish(marker_msg)

        rospy.sleep(rospy.Duration(0.1))
