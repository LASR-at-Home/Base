#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
import numpy as np

def get_laser_and_publish_pcl(msg):
    pcl_msg = lg.LaserProjection().projectLaser(msg)
    pcl_points = [p for p in pc2.read_points(pcl_msg, field_names=("x, y, z"), skip_nans=True)]
    
    padded_points = []
    for point in pcl_points:
        for z in np.linspace(0., 2., 10):
            padded_points.append((point[0], point[1], z))

    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = "base_link"
    padded_pcl = pc2.create_cloud_xyz32(h, padded_points)
    pub.publish(padded_pcl)

if __name__ == "__main__":
    rospy.init_node("laser_to_pointcloud_node")
    pub = rospy.Publisher("/pcl_from_laser_debug", PointCloud2, queue_size=10)
    sub = rospy.Subscriber("/scan", LaserScan, get_laser_and_publish_pcl)
    while not rospy.is_shutdown():
        rospy.spin()