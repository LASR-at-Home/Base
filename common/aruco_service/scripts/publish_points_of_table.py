#!/usr/bin/env python

import yaml
import rospy
import datetime
from aruco_service.srv import TableNumber, TableNumberResponse
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import rosparam
import tf2_ros
import tf2_geometry_msgs

# TODO: Load filename from rosparam, with default being this one.
FILENAME = "/home/peter/robocup_ws/src/Base/common/aruco_service/test_check_table_sim.yaml"

def create_marker_msg(point, idx):
    marker_msg = Marker()
    marker_msg.header.frame_id = "map"
    marker_msg.header.stamp = rospy.Time.now()
    marker_msg.id = idx
    marker_msg.type = Marker.SPHERE
    marker_msg.action = Marker.ADD
    marker_msg.pose.position.x = point[0]
    marker_msg.pose.position.y = point[1]
    marker_msg.color.r = 1.0
    marker_msg.color.g = 0.0
    marker_msg.color.b = 0.0
    marker_msg.scale.x = 0.1
    marker_msg.scale.y = 0.1
    marker_msg.scale.z = 0.1
    marker_msg.color.a = 1.0
    return marker_msg

def publish_points(number):
    table = number.table
    seconds = rospy.get_time()
    objects_marker_pub = rospy.Publisher("/table/objects_cuboid", Marker, queue_size=4)
    persons_marker_pub = rospy.Publisher("/table/persons_cuboid", Marker, queue_size=4)

    obj = rospy.get_param("/tables/table" + str(table) + "/objects_cuboid")
    
    objects_marker_pub.publish(create_marker_msg(obj[0], 0))
    objects_marker_pub.publish(create_marker_msg(obj[1], 1))
    objects_marker_pub.publish(create_marker_msg(obj[2], 2))
    objects_marker_pub.publish(create_marker_msg(obj[3], 3))

    if table >= 0:
        per = rospy.get_param("/tables/table" + str(table) + "/persons_cuboid")

        persons_marker_pub.publish(create_marker_msg(per[0], 0))
        persons_marker_pub.publish(create_marker_msg(per[1], 1))
        persons_marker_pub.publish(create_marker_msg(per[2], 2))
        persons_marker_pub.publish(create_marker_msg(per[3], 3))

    rospy.loginfo("Published points for table " + str(table))

    return TableNumberResponse(True)

if __name__ == "__main__":

    rospy.init_node("point_publisher")
    s = rospy.Service("publish_table_points", TableNumber, publish_points)
    
    rospy.loginfo("Point Publisher Service Ready")
    
    while not rospy.is_shutdown():
        rospy.sleep(1.0)