#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
import numpy as np
import tf2_ros
import tf2_geometry_msgs


def get_transform(from_frame, to_frame):
    try:
        t = tf_buffer.lookup_transform(
            to_frame, from_frame, rospy.Time(0), rospy.Duration(0.5)
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


def get_pcl_from_laser(msg):
    """Converts a LaserScan message to a padded point cloud in camera frame.

    Args:
        msg (LaserScan): ROS Laser Scan message from /scan topic.
    Returns:
        PointCloud2: ROS PointCloud2 message with padded points in camera frame.
    """
    pcl_msg = lg.LaserProjection().projectLaser(msg)
    pcl_points = [
        p for p in pc2.read_points(pcl_msg, field_names=("x, y, z"), skip_nans=True)
    ]
    tr = get_transform("base_laser_link", "xtion_rgb_optical_frame")

    padded_converted_points = []
    for point in pcl_points:
        for z in np.linspace(0.0, 2.0, 10):
            p = apply_transform((point[0], point[1], z), tr)
            padded_converted_points.append(p)

    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = "xtion_rgb_optical_frame"
    padded_pcl = pc2.create_cloud_xyz32(h, padded_converted_points)

    return padded_pcl


def get_laser_and_publish_pcl(msg):
    padded_pcl = get_pcl_from_laser(msg)
    pub.publish(padded_pcl)


if __name__ == "__main__":
    rospy.init_node("laser_to_pointcloud_node")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    pub = rospy.Publisher("/pcl_from_laser_debug", PointCloud2, queue_size=10)
    sub = rospy.Subscriber("/scan", LaserScan, get_laser_and_publish_pcl)
    while not rospy.is_shutdown():
        rospy.spin()
