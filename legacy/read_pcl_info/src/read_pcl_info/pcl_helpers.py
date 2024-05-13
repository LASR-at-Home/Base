#!/usr/bin/env python
from std_msgs.msg import Bool, String, Header
import rospy
from visualization_msgs.msg import Marker
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Point, PointStamped
from math import sqrt, cos, sin
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
from tf_module.srv import BaseTransformRequest, ApplyTransformRequest, LatestTransformRequest, BaseTransform, \
    LatestTransform, ApplyTransform
from lift.defaults import PUBLISH_MARKERS
from geometry_msgs.msg import Pose
from markers.markers_helpers import create_point_marker
from image_geometry import PinholeCameraModel
# from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from geometry_msgs.msg import Pose, Point, Quaternion

MEAN_DISTANCE_THRESHOLD = 0.5
DIST_THRESHOLD = 0.1

def get_pose_from_param(name):
    if not rospy.has_param(name):
        return None
    pose = rospy.get_param(name)
    print(pose)
    return Pose(Point(pose['position']['x'],
                      pose['position']['y'],
                      pose['position']['z']),
                Quaternion(pose['orientation']['x'],
                           pose['orientation']['y'],
                           pose['orientation']['z'],
                           pose['orientation']['w']))

object_pos = get_pose_from_param("/door/pose")
door = (object_pos.position.x, object_pos.position.y)
# door = rospy.get_param('door_location')
print("door location: {}".format(door))
door_location = Point(x=door[0], y=door[1])

object_location = door_location
door_detected = False
prev_door_detected = False

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
    """
    middle_part = laser_scan.ranges[len(laser_scan.ranges) // 3: 2 * len(laser_scan.ranges) // 3]
    filtered_ranges = [np.nan] * len(laser_scan.ranges)
    filtered_ranges[len(laser_scan.ranges) // 3: 2 * len(laser_scan.ranges) // 3] = middle_part
    mean_distance = np.nanmean(filtered_ranges)

    return mean_distance, filtered_ranges


def limit_laser_scan(laser_scan, is_pub_markers=True, is_publish=False, is_pcl=False):
    """
        Filter the laser scan data based on angle and range and publish it
    """
    # def limit_laser_scan(laser_scan, tf_base,tf_latest, tf_apply, tf, is_pub_markers=True, is_publish=False, is_pcl=False):
    print("Door detected!{}".format(door_detected))
    mean_distance, filtered_ranges = filter_laser_scan(laser_scan)
    limited_scan = laser_scan

    # update the laser scan
    limited_scan.header.stamp = rospy.Time.now()
    limited_scan.ranges = filtered_ranges

    is_door_detected(limited_scan)
    # is_door_detected(limited_scan, tf_base)

    if is_publish:
        pub_filtered.publish(limited_scan)
        rospy.loginfo("published the filtered laser scan")
        rospy.sleep(1)

    if is_pcl:
        padded_pcl = get_pcl_from_laser(limited_scan)
        poses = transform_pointcloud_to_map(padded_pcl)
        pub_markers(poses, is_point=False)
        pub_pcl.publish(padded_pcl)



def laser_callback(laser_data):
    global door_detected, object_location
    # Filter the laser scan data based on angle and range
    mean, filtered_points = filter_laser_scan(laser_data)
    print("filtered_points: {}".format(filtered_points))

    final_points = []
    for i, range_val in enumerate(filtered_points):
        # print(f"i: {i}, range_val: {range_val}")
        angle = laser_data.angle_min + i * laser_data.angle_increment
        x = range_val * cos(angle)
        y = range_val * sin(angle)
        final_points.append((x, y, angle))

    print("final_points: {}".format(final_points))

    for i, range_val in enumerate(final_points):
        print("i: {}, range_val: {}, testing".format(i, range_val))
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


def laser_scan_to_points(laser_data):
    # def laser_scan_to_points(laser_data, tf_base):
    req = BaseTransformRequest()
    req.frame = String('base_laser_link')
    req.target_frame = String('map')

    points = []
    for i, range_val in enumerate(laser_data.ranges):
        if not np.isnan(range_val):
            angle = laser_data.angle_min + i * laser_data.angle_increment

            x = range_val * cos(angle)
            y = range_val * sin(angle)

            points.append(Point(x=x, y=y, z=0.0))

    req.points = points
    points = tf_base(req)
    # print("points here mmfmfmf: {}".format(points))
    points_in_map_frame = [p.point for p in points.new_points]

    return points_in_map_frame


def pub_markers(points, is_point=True):
    if is_point:
        for i in range(len(points)):
            door_pose_laser.publish(create_point_marker(points[i].x, points[i].y, points[i].z, i))
    else:
        for i, pose in enumerate(points):
            pcl_to_point.publish(create_point_marker(pose.position.x, pose.position.y, pose.position.z, i))

def is_door_detected(laser_data):
    global door_detected, door_location, prev_door_detected

    # def is_door_detected(laser_data, tf_base):
    # points = laser_scan_to_points(laser_data, tf_base)
    points = laser_scan_to_points(laser_data)

    # if PUBLISH_MARKERS:
    #     pub_markers(points)

    for point in points:
        x = point.x
        y = point.y
        dist = sqrt((x - door_location.x) ** 2 + (y - door_location.y) ** 2)

        if dist < DIST_THRESHOLD:
            door_detected = True
            rospy.set_param("/door_detected_official", True)
            door_pose.publish(True)
            break
    else:
        door_pose.publish(False)
        rospy.set_param("/door_detected_official", False)
        door_detected = False

    if door_detected != prev_door_detected:
        rospy.loginfo("Door state changed: {} -> {}".format(prev_door_detected, door_detected))

    prev_door_detected = door_detected

def get_pcl_from_laser(msg):
    """ Converts a LaserScan message to a collection of points in camera frame, with their corresponding pixel values in a flat array. The method pads out the points to add vertical "pillars" to the point cloud.

    Args:
        msg (LaserScan): ROS Laser Scan message from /scan topic.
    Returns:
        List: A list of tuples containing all the filtered and padded points in camera frame.
        List: A list of pixel values corresponding to the points in the first list.
    """
    # First get the laser scan points, and then convert to camera frame
    pcl_msg = lg.LaserProjection().projectLaser(msg)
    pcl_points = [p for p in pc2.read_points(pcl_msg, field_names=("x, y, z"), skip_nans=True)]

    tf_req = LatestTransformRequest()
    tf_req.target_frame = "xtion_rgb_optical_frame"
    tf_req.from_frame = "base_laser_link"
    t = tf_latest(tf_req)

    padded_points = []
    for point in pcl_points:
        # Pad out the points to add vertical "pillars" to the point cloud
        for z in np.linspace(0., 1., 5):
            padded_points.append(Point(x=point[0], y=point[1], z=z))

    apply_req = ApplyTransformRequest()
    apply_req.points = padded_points
    apply_req.transform = t.transform
    res = tf_apply(apply_req)

    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = "xtion_rgb_optical_frame"
    r = [(p.x, p.y, p.z) for p in res.new_points]
    padded_pcl = pc2.create_cloud_xyz32(h, r)

    return padded_pcl





def transform_pointcloud_to_map(pcl, source_frame="xtion_rgb_optical_frame"):
    pss = []
    for point in point_cloud2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=True):
        pss.append(Point(x=point[0], y=point[1], z=point[2]))

    req = BaseTransformRequest()
    req.frame = String(source_frame)
    req.target_frame = String('map')
    req.points = pss

    points_map = tf_base(req)

    poses_in_map_frame = []
    for point_map in points_map.new_points:
        pose_in_map_frame = Pose()
        pose_in_map_frame.position = point_map.point
        poses_in_map_frame.append(pose_in_map_frame)

    return poses_in_map_frame


def door_detected_callback(msg):
    global door_detected
    door_detected = msg.data
    # rospy.loginfo("the door detected {}".format(door_detected))


if __name__ == "__main__":
    rospy.init_node("pcl_helpers")
    print("Waiting for the door location")
    camera = PinholeCameraModel()

    # tf
    tf_base = rospy.ServiceProxy('base_transform', BaseTransform)
    tf_latest = rospy.ServiceProxy('latest_transform', LatestTransform)
    tf_apply = rospy.ServiceProxy('apply_transform', ApplyTransform)

    # door detected
    door_pose = rospy.Publisher("/door_detected", Bool, queue_size=100)
    door_pose_laser = rospy.Publisher("/door_pose_laser", Marker, queue_size=100)
    sub_door_detected = rospy.Subscriber('/door_detected', Bool, door_detected_callback)

    pcl_to_point = rospy.Publisher("/pcl_to_point", Marker, queue_size=100)
    pub_pcl = rospy.Publisher("/pcl_from_laser_debug", PointCloud2, queue_size=10)

    # laser
    pub_filtered = rospy.Publisher('/filtered_laser_scan', LaserScan, queue_size=10)
    laser = rospy.wait_for_message('/scan', LaserScan)
    limit_laser_scan(laser)
    print(door_detected, " door_detected++")
    while True:
        laser = rospy.wait_for_message('/scan', LaserScan)
        limit_laser_scan(laser)
        print(door_detected, " door_detected----")
        rospy.sleep(5)
    # sub_laser = rospy.Subscriber('/scan', LaserScan, limit_laser_scan)
    # pcl

    while not rospy.is_shutdown():
        rospy.spin()
