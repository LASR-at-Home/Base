#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Point, PointStamped
from math import sqrt, cos, sin
import tf2_ros
from std_msgs.msg import Bool
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
# from tf_module.tf_transforms_base import transform_point, apply_transform, get_transform, transform_point_2, transform_point_from_given
from tf_module.srv import BaseTransformRequest, ApplyTransformRequest, LatestTransformRequest, BaseTransform, LatestTransform, ApplyTransform
from lift.defaults import PUBLISH_MARKERS
from geometry_msgs.msg import Pose
from markers.markers_helpers import create_point_marker
from image_geometry import PinholeCameraModel

MEAN_DISTANCE_THRESHOLD = 0.5
DIST_THRESHOLD = 0.1
# sim
door_location = Point(x=-5.3076634407,y=-1.26610350609)

# door_location = Point(x=3.78017091751, y=-0.921108067036)
object_location = None
door_detected = False
prev_door_detected = False
camera = PinholeCameraModel()

# set up the tf module


# get pointcloud
def get_pointcloud():
    pointcloud = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
    return pointcloud


# get the laser scan
def get_laser_scan():
    laser_scan = rospy.wait_for_message('/scan', LaserScan)
    return laser_scan


def filter_laser_scan(laser_scan):
    """
    Filter the laser scan data based on angle and range
    Parameters
    ----------
    laser_scan

    Returns
    -------

    """
    middle_part = laser_scan.ranges[len(laser_scan.ranges) // 3: 2 * len(laser_scan.ranges) // 3]
    filtered_ranges = [np.nan] * len(laser_scan.ranges)
    filtered_ranges[len(laser_scan.ranges) // 3: 2 * len(laser_scan.ranges) // 3] = middle_part
    mean_distance = np.nanmean(filtered_ranges)

    return mean_distance, filtered_ranges


def limit_laser_scan(laser_scan, is_pub_markers=True, is_publish=False, is_pcl=False):
    """
    Filter the laser scan data based on angle and range and publish it
    Parameters
    ----------
    laser_scan
    is_publish
    is_pcl

    Returns
    -------

    """
    print("Door detected!{}".format(door_detected))
    mean_distance, filtered_ranges = filter_laser_scan(laser_scan)
    limited_scan = laser_scan

    # update the laser scan
    limited_scan.header.stamp = rospy.Time.now()
    limited_scan.ranges = filtered_ranges

    if is_pub_markers:
        pub_laser_markers(limited_scan)

    if is_publish:
        pub.publish(limited_scan)
        rospy.loginfo("published the filtered laser scan")
        rospy.sleep(1)

    if is_pcl:
        padded_pcl = get_pcl_from_laser(limited_scan)
        poses = transform_pointcloud_to_map(padded_pcl)
        for i, pose in enumerate(poses):
            pcl_to_point.publish(create_point_marker(pose.position.x, pose.position.y, pose.position.z, i))

        pub.publish(padded_pcl)


def laser_callback(laser_data):
    global door_detected, object_location
    # Filter the laser scan data based on angle and range
    mean, filtered_points = filter_laser_scan(laser_data)
    print("filtered_points: {}".format(filtered_points))

    # set up the tf module
    rospy.wait_for_service('base_transform')



    final_points = []
    for i, range_val in enumerate(filtered_points):
        # print(f"i: {i}, range_val: {range_val}")
        angle = laser_data.angle_min + i * laser_data.angle_increment
        x = range_val * cos(angle)
        y = range_val * sin(angle)
        final_points.append((x, y, angle))

    for i, range_val in enumerate(final_points):
        req = BaseTransformRequest()
        req.point = Point(x=range_val[0], y=range_val[1], z=0.0)
        req.frame = String('base_laser_link')
        req.target_frame = String('map')

        x, y, z = tf_base(req)
        door_pose_laser.publish(create_point_marker(x, y, 0.0, i))

    to_present = []

    for x, y, angle in final_points:
        to_present.append((x, y))

        dist = sqrt((x - object_location.x) ** 2 + (y - object_location.y) ** 2)

        print("dist: {dist}".format(dist))
        if dist < DIST_THRESHOLD:
            door_detected = True
            print("Door detected!{}".format(door_detected))
            break

    if PUBLISH_MARKERS:
        for i, point in enumerate(to_present):
            if point[0] != np.nan:
                req = BaseTransformRequest()
                req.point = Point(x=point[0], y=point[1], z=0.0)
                req.frame = "base_laser_link"
                req.target_frame = "map"
                x, y, z = tf_base(req)
                door_pose_laser.publish(create_point_marker(x, y, z, i))

def point_cloud_callback(pcl_data):
    # Filter the point cloud data based on range and height
    filtered_points = []
    for point in point_cloud2.read_points(pcl_data, field_names=("x", "y", "z"), skip_nans=True):
        dist = sqrt((point[0] - door_location.x) ** 2 + (point[1] - door_location.y) ** 2)
        if dist < 0.1 and abs(point[2] - 1.0) < 0.05:  # Adjust the thresholds as needed
            filtered_points.append(point)

    # Check if any points match the expected door location
    if filtered_points:
        door_detected = True

from std_msgs.msg import String

def laser_scan_to_points(laser_data):
    points_in_map_frame = []

    for i, range_val in enumerate(laser_data.ranges):
        angle = laser_data.angle_min + i * laser_data.angle_increment

        x = range_val * cos(angle)
        y = range_val * sin(angle)

        # set up the tf module
        # rospy.wait_for_service('base_transform')
        req = BaseTransformRequest()
        req.point = Point(x=x, y=y, z=0.0)
        req.frame = String('base_laser_link')
        req.target_frame = String('map')

        points = tf_base(req)

        points_in_map_frame.append(points.point)

    return points_in_map_frame

def pub_markers(points):
    for i in range(len(points)):
        door_pose_laser.publish(create_point_marker(points[i].x, points[i].y, points[i].z, i))

def pub_laser_markers(laser_data):
    global door_detected, object_location, prev_door_detected

    points = laser_scan_to_points(laser_data)
    pub_markers(points)

    for point in points:
        x = point.x
        y = point.y
        dist = sqrt((x - object_location.x) ** 2 + (y - object_location.y) ** 2)

        if dist < 0.1:
            door_detected = True
            door_pose.publish(True)
            break
    else:
        door_pose.publish(False)
        door_detected = False

    if door_detected != prev_door_detected:
        rospy.loginfo("Door state changed: {} -> {}".format(prev_door_detected, door_detected))

    prev_door_detected = door_detected


def get_pcl_from_laser(msg):
    """
    Get the point cloud from the laser scan
    Parameters
    ----------
    msg

    Returns
    -------

    """
    pcl_msg = lg.LaserProjection().projectLaser(msg)
    pcl_points = [p for p in pc2.read_points(pcl_msg, field_names=("x, y, z"), skip_nans=True)]

    tf_req = LatestTransformRequest()
    tf_req.from_frame = "base_laser_link"
    tf_req.target_frame = "xtion_rgb_optical_frame"
    tr = tf_latest(tf_req)

    padded_points = []
    pixels = []
    for point in pcl_points:
        # Pad out the points to add vertical "pillars" to the point cloud
        for z in np.linspace(0., 1., 5):
            padded_points.append(Point(x=point[0], y=point[1], z=z))

    apply_req = ApplyTransformRequest()
    apply_req.points = padded_points
    apply_req.transform = t.transform
    res = tf_apply(apply_req)

    padded_converted_points = []
    for p in res.new_points:
        pt = (p.x, p.y, p.z)
        u, v = camera.project3dToPixel(pt)
        # Filter out points that are outside the camera frame
        if u >= 0 and u < 640 and v >= 0 and v < 480:
            pixels.append(u)
            pixels.append(v)
            padded_converted_points.append(pt)

    return padded_converted_points, pixels



def transform_pointcloud_to_map(pcl, source_frame="xtion_rgb_optical_frame"):
    poses_in_map_frame = []

    for point in point_cloud2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=True):
        point_source = PointStamped()
        point_source.header.frame_id = source_frame
        point_source.header.stamp = rospy.Time(0)
        point_source.point.x = point[0]
        point_source.point.y = point[1]
        point_source.point.z = point[2]

        try:
            point_map = transform_point(point_source=point_source, source_frame=source_frame, to_frame="map")

        except IndexError as e:
            rospy.logwarn("Failed to transform point in PointCloud, warning: %s", str(e))
            point_map = None

        pose_in_map_frame = Pose()
        pose_in_map_frame.position = point_map.point
        poses_in_map_frame.append(pose_in_map_frame)

    return poses_in_map_frame

def door_detected_callback(msg):
    global door_detected
    door_detected = msg.data
    rospy.logerr("the door detected {}".format(door_detected))

if __name__ == "__main__":
    rospy.init_node("pcl_helpers")
    print("Waiting for the door location")

    tf_base = rospy.ServiceProxy('base_transform', BaseTransform)
    tf_latest = rospy.ServiceProxy('latest_transform', LatestTransform)


    pcl_to_point = rospy.Publisher("/pcl_to_point", Marker, queue_size=100)
    pub = rospy.Publisher('/filtered_laser_scan', LaserScan, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, limit_laser_scan)
    door_pose_laser = rospy.Publisher("/door_pose_laser", Marker, queue_size=100)
    door_pose = rospy.Publisher("/door_detected", Bool, queue_size=100)

    while not rospy.is_shutdown():
        rospy.spin()
