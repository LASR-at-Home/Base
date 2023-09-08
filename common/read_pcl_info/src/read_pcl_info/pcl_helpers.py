#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
import numpy as np
from std_msgs.msg import Header


MEAN_DISTANCE_THRESHOLD = 0.5

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


# get pointcloud
def get_pointcloud():
    pointcloud = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
    return pointcloud
# get the laser scan
def get_laser_scan():
    laser_scan = rospy.wait_for_message('/scan', LaserScan)
    return laser_scan

def filter_laser_scan(laser_scan, filter_range=3):
    filtered_ranges = laser_scan.ranges[len(laser_scan.ranges) // filter_range: 2 * len(laser_scan.ranges) // filter_range]
    mean_distance = np.nanmean(filtered_ranges)
    return mean_distance, filtered_ranges


def limit_laser_scan(laser_scan):
    mean_distance, filtered_ranges = filter_laser_scan(laser_scan)
    # to_pub = LaserScan()
    # h = Header()
    # h.frame_id = laser_scan.header.frame_id
    # h.stamp = rospy.Time.now()
    #
    # to_pub.header = h
    # to_pub.header.stamp = rospy.Time.now()
    # to_pub.ranges = filtered_ranges
    # pub.publish(to_pub)

    limited_scan = laser_scan

    middle_part = laser_scan.ranges[len(laser_scan.ranges) // 3: 2 * len(laser_scan.ranges) // 3]
    limited = [np.nan] * len(limited_scan.ranges)
    limited[len(laser_scan.ranges) // 3: 2 * len(laser_scan.ranges) // 3] = middle_part

    # update
    limited_scan.header.stamp = rospy.Time.now()
    limited_scan.ranges = limited

    pub.publish(limited_scan)
    rospy.loginfo("published the filtered laser scan")
    rospy.sleep(1)

# visualise

# this is with an offset

# # Calculate new angle-related fields
# angle_min_new = laser_scan.angle_min + (len(limited) // 3) * laser_scan.angle_increment
# angle_max_new = laser_scan.angle_min + ((2 * len(limited) // 3) - 1) * laser_scan.angle_increment
# angle_increment_new = laser_scan.angle_increment
#
# # Update angle-related fields
# limited_scan.angle_min = angle_min_new
# limited_scan.angle_max = angle_max_new
# limited_scan.angle_increment = angle_increment_new
# limited_scan.ranges = limited

#
if __name__ == "__main__":
    rospy.init_node("pcl_helpers")
    # publish_laser_topic()
    pub = rospy.Publisher('/filtered_laser_scan', LaserScan, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, limit_laser_scan)
    while not rospy.is_shutdown():
        rospy.spin()

