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
import tf2_geometry_msgs

MEAN_DISTANCE_THRESHOLD = 0.5

#sim
# door_location = Point(x=-5.379326820373535, y=-1.0147719383239746)
# real
door_location = Point(x= 3.78017091751, y= -0.921108067036)
door_detected = False
prev_door_detected = False


def create_point_marker(x, y, z, idx):
    marker_msg = Marker()
    marker_msg.header.frame_id = "map"
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


# get pointcloud
def get_pointcloud():
    pointcloud = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
    return pointcloud


# get the laser scan
def get_laser_scan():
    laser_scan = rospy.wait_for_message('/scan', LaserScan)
    return laser_scan


def filter_laser_scan(laser_scan):
    middle_part = laser_scan.ranges[len(laser_scan.ranges) // 3: 2 * len(laser_scan.ranges) // 3]
    filtered_ranges = [np.nan] * len(laser_scan.ranges)
    filtered_ranges[len(laser_scan.ranges) // 3: 2 * len(laser_scan.ranges) // 3] = middle_part
    mean_distance = np.nanmean(filtered_ranges)
    return mean_distance, filtered_ranges


def limit_laser_scan(laser_scan, is_publish=False):
    print("Door detected!{}".format(door_detected))
    mean_distance, filtered_ranges = filter_laser_scan(laser_scan)
    limited_scan = laser_scan

    # update the laser scan
    limited_scan.header.stamp = rospy.Time.now()
    limited_scan.ranges = filtered_ranges
    pub_markers(limited_scan)

    # if is_publish:
    #     pub.publish(limited_scan)
    #     rospy.loginfo("published the filtered laser scan")
    #     rospy.sleep(1)

    # for the last code
    # padded_pcl = get_pcl_from_laser(limited_scan)
    # poses = transform_pointcloud_to_map(padded_pcl)
    # for i, pose in enumerate(poses):
    #     pcl_to_point.publish(create_point_marker(pose.position.x, pose.position.y, pose.position.z, i))
    #
    #
    # pub.publish(padded_pcl)



# for the last code
# def get_transform(from_frame, to_frame):
#     try:
#         t = tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(0.5))
#         return t
#     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#         raise
#
# def apply_transform(input_xyz, transform, target="xtion_rgb_optical_frame"):
#     ps = PointStamped()
#     ps.point.x = input_xyz[0]
#     ps.point.y = input_xyz[1]
#     ps.point.z = input_xyz[2]
#     ps.header.frame_id = target
#     ps.header.stamp = rospy.Time.now()
#
#     tr_point = tf2_geometry_msgs.do_transform_point(ps, transform)
#     return (tr_point.point.x, tr_point.point.y, tr_point.point.z)
# for the last code end

def get_transform(from_frame, to_frame):
    try:
        t = tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(0.5))
        return t
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

def apply_transform(ps, transform):
    tr_point = tf2_geometry_msgs.do_transform_point(ps, transform)
    # return (tr_point.point.x, tr_point.point.y, tr_point.point.z)
    return tr_point

def transform_point(x_, y_):
    point = PointStamped()
    point.header.stamp = rospy.Time.now()
    point.header.frame_id = "map"
    point.point.x = x_
    point.point.y = y_
    point.point.z = 0.0

    tr = get_transform("base_laser_link", "map")
    x, y, z = apply_transform(point, tr)
    # return Point(x=x, y=y, z=z)
    return x, y, z

def laser_callback(laser_data):
    global door_detected, door_location
    # Filter the laser scan data based on angle and range
    mean, filtered_points = filter_laser_scan(laser_data)
    print("filtered_points: {}".format(filtered_points))

    # filtered_points = []
    # for angle, range_val in zip(laser_data.angle_increment, laser_data.ranges):
    #     if abs(angle) < 0.1 and range_val < 2.0:  # Define angle and range thresholds
    #         filtered_points.append((range_val, angle))

    print(laser_data.header.frame_id)

    final_points = []
    for i, range_val in enumerate(filtered_points):
        # print(f"i: {i}, range_val: {range_val}")
        angle = laser_data.angle_min + i * laser_data.angle_increment
        x = range_val * cos(angle)
        y = range_val * sin(angle)
        final_points.append((x, y, angle))

    for i, range_val in enumerate(final_points):
        x, y, z = transform_point(range_val[0], range_val[1])
        door_pose_laser.publish(create_point_marker(x, y, 0.0, i))

    to_present = []

    for x, y, angle in final_points:
        to_present.append((x, y))

        dist = sqrt((x - door_location.x) ** 2 + (y - door_location.y) ** 2)
        # else:
        #     dist = np.inf

        print("dist: {dist}".format(dist))
        # print(f"Door detected!{door_detected}")
        if dist < 0.1:  # Adjust the threshold as needed
            door_detected = True
            print("Door detected!{}".format(door_detected))
            break

    # for i, point in enumerate(to_present):
    #     if point[0] != np.nan:
            # x, y, z = transform_point(point)
            # door_pose_laser.publish(create_point_marker(x, y, z, i))

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
    # laser_data = rospy.wait_for_message('/scan', LaserScan)

    points_in_map_frame = []

    for i, range_val in enumerate(laser_data.ranges):
        angle = laser_data.angle_min + i * laser_data.angle_increment

        x = range_val * cos(angle)
        y = range_val * sin(angle)

        point_laser = PointStamped()
        point_laser.header.stamp = rospy.Time.now()
        point_laser.header.frame_id = "base_laser_link"
        point_laser.point.x = x
        point_laser.point.y = y
        point_laser.point.z = 0.0

        # points_in_map_frame.append(point_laser.point)
        try:
            tr = get_transform("base_laser_link", "map")
            points = apply_transform(point_laser, tr)
            points_in_map_frame.append(points.point)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to transform point.")
            rospy.logwarn(e)

    return points_in_map_frame

def pub_markers(laser_data):
    global door_detected, door_location, prev_door_detected
    points = laser_scan_to_points(laser_data)
    for i in range(len(points)):
        door_pose_laser.publish(create_point_marker(points[i].x, points[i].y, points[i].z, i))

    

    for point in points:
        x = point.x
        y = point.y
        dist = sqrt((x - door_location.x) ** 2 + (y - door_location.y) ** 2)

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




import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2

def get_pcl_from_laser(msg):
    """ Converts a LaserScan message to a padded point cloud in camera frame.
    Args:
        msg (LaserScan): ROS Laser Scan message from /scan topic.
    Returns:
        PointCloud2: ROS PointCloud2 message with padded points in camera frame.
    """
    pcl_msg = lg.LaserProjection().projectLaser(msg)
    pcl_points = [p for p in pc2.read_points(pcl_msg, field_names=("x, y, z"), skip_nans=True)]
    tr = get_transform("base_laser_link", "xtion_rgb_optical_frame")

    padded_converted_points = []
    for point in pcl_points:
        for z in np.linspace(0., 2., 10):
            p = apply_transform((point[0], point[1], z), tr)
            padded_converted_points.append(p)

    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = "xtion_rgb_optical_frame"
    padded_pcl = pc2.create_cloud_xyz32(h, padded_converted_points)

    return padded_pcl


from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import Pose

def transform_pointcloud_to_map(pcl):
    source_frame = "xtion_rgb_optical_frame"
    poses_in_map_frame = []

    for point in point_cloud2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=True):
        point_source = PointStamped()
        point_source.header.frame_id = source_frame
        point_source.header.stamp = rospy.Time(0)
        point_source.point.x = point[0]
        point_source.point.y = point[1]
        point_source.point.z = point[2]

        try:
            # Transform the point from the source frame to the map frame
            point_map = tf_buffer.transform(point_source, "map")
            print("point_map: {}".format(point_map))

            # Create a Pose from the transformed point
            pose_in_map_frame = Pose()
            pose_in_map_frame.position = point_map.point
            poses_in_map_frame.append(pose_in_map_frame)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to transform point in PointCloud.")
            rospy.logwarn("Extrapolation warning: %s", str(e))
            # Wait for updated transformation data with a timeout
            tf_buffer.can_transform("map", source_frame, rospy.Time(0), rospy.Duration(4.0))
            # Retry the transformation after waiting
            point_map = tf_buffer.transform(point_source, "map")
            pose_in_map_frame = Pose()
            pose_in_map_frame.position = point_map.point
            poses_in_map_frame.append(pose_in_map_frame)

    # Now, 'poses_in_map_frame' contains Poses of the points in the map frame
    return poses_in_map_frame

    # try:

        # tf_buffer.can_transform("map", source_frame, rospy.Time(0), rospy.Duration(4.0))
        #
        # # Extract the position and orientation from the transformation
        # map_pose = tf_buffer.lookup_transform("map", source_frame, rospy.Time(0), rospy.Duration(4.0))
        #
        # # Create a Pose from the transformed position and orientation
        # map_position = map_pose.transform.translation
        # map_orientation = map_pose.transform.rotation

        # return Pose(position=map_position, orientation=map_orientation)
    # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #     rospy.logwarn("Failed to transform point cloud.")


from std_msgs.msg import Bool

if __name__ == "__main__":
    rospy.init_node("pcl_helpers")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    # pub = rospy.Publisher('/filtered_laser_scan', LaserScan, queue_size=10)
    # laser_sub = rospy.Subscriber('/scan', LaserScan, laser_callback)
    # laser_sub = rospy.Subscriber('/scan', LaserScan, limit_laser_scan)
    # door_pose_laser = rospy.Publisher("/door_pose_laser", Marker, queue_size=100)
    # door_pose = rospy.Publisher("/pose_point", Marker, queue_size=100)
    # pcl_sub = rospy.Subscriber('/xtion/depth_registered/points', PointCloud2, point_cloud_callback)

    sub = rospy.Subscriber('/scan', LaserScan, limit_laser_scan)
    door_pose_laser = rospy.Publisher("/door_pose_laser", Marker, queue_size=100)
    door_pose = rospy.Publisher("/door_detected", Bool, queue_size=100)


    # tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    # tf_listener = tf2_ros.TransformListener(tf_buffer)
    # pub = rospy.Publisher("/pcl_from_laser_debug", PointCloud2, queue_size=10)
    # sub = rospy.Subscriber('/scan', LaserScan, limit_laser_scan)
    # pcl_to_point = rospy.Publisher("/pcl_to_point", Marker, queue_size=100)


    while not rospy.is_shutdown():
        rospy.spin()
