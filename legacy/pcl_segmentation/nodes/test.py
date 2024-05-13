#!/usr/bin/env python3
import rospy
from pcl_segmentation.srv import SegmentCuboid
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from cv_bridge3 import CvBridge, cv2
from common_math import pcl_msg_to_cv2
if __name__ == "__main__":
    rospy.init_node("test_segment")
    low = Point(1.34, -1.59, 0.0)
    high = Point(2.88, -0.61, 8.0)
    segment = rospy.ServiceProxy("/pcl_segmentation_server/segment_cuboid", SegmentCuboid)
    bridge = CvBridge()

    while True:
        pcl = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        im_before = pcl_msg_to_cv2(pcl)

        result = segment(pcl, low, high).mask
        mask = bridge.imgmsg_to_cv2_np(result) 
        cv2.imshow("mask", mask)
        cv2.waitKey(1)
        cv2.imshow("applied mask", cv2.bitwise_and(im_before,im_before,mask = mask))
        cv2.waitKey(1)
