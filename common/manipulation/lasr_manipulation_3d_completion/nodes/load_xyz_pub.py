#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2


def load_xyz(path):
    try:
        return np.loadtxt(path, dtype=np.float32)
    except Exception as e:
        rospy.logerr(f"Failed to load .xyz file: {e}")
        rospy.signal_shutdown("XYZ file loading failed.")
        return np.array([])


def publish_xyz():
    rospy.init_node("xyz_publisher", anonymous=True)

    xyz_path = rospy.get_param("xyz_path", None)
    rospy.loginfo(f"Loading XYZ file from: {xyz_path}")
    if xyz_path is None:
        rospy.logerr("Please set xyz_path parameter to point to your .xyz file.")
        return

    pts = load_xyz(xyz_path)
    if pts.size == 0:
        rospy.logerr("Loaded point cloud is empty.")
        return

    pub = rospy.Publisher("/partial_cloud_vis", PointCloud2, queue_size=1)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "base_link"
        msg = pc2.create_cloud_xyz32(header, pts)
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    publish_xyz()
