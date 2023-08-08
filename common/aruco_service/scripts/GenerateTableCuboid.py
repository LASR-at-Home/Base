#!/usr/bin/env python

import yaml
import rospy
import datetime
from aruco_service.srv import TableNumber, TableNumberResponse
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import rosparam

# Units are in meters
# The marker should be placed on the bottom left corner of the table, with the x axis pointing to the right and the y axis pointing up
TABLE_LONG_SIDE = 1.2
TABLE_SHORT_SIDE = 0.6
PADDING = 0.5
FILENAME = "/home/peter/robocup_ws/src/aruco_service/test_check_table_sim.yaml"

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

def generate_table_cuboid(number):
    table = number.table
    seconds = rospy.get_time()
    objects_marker_pub = rospy.Publisher("/table/objects_cuboid", Marker, queue_size=10)
    persons_marker_pub = rospy.Publisher("/table/persons_cuboid", Marker, queue_size=10)

    if latest_pose is not None and (seconds - latest_pose.header.stamp.secs < 1.0):

        corner_1 = [latest_pose.pose.position.x - 0.05, latest_pose.pose.position.y - 0.05] # subtract 5cm because of marker size to get corner
        corner_2 = [latest_pose.pose.position.x - 0.05 + TABLE_LONG_SIDE, latest_pose.pose.position.y - 0.05]
        corner_3 = [latest_pose.pose.position.x - 0.05 + TABLE_LONG_SIDE, latest_pose.pose.position.y - 0.05 + TABLE_SHORT_SIDE]
        corner_4 = [latest_pose.pose.position.x - 0.05, latest_pose.pose.position.y - 0.05 + TABLE_SHORT_SIDE]
        objects_marker_pub.publish(create_marker_msg(corner_1, 0))
        objects_marker_pub.publish(create_marker_msg(corner_2, 1))
        objects_marker_pub.publish(create_marker_msg(corner_3, 2))
        objects_marker_pub.publish(create_marker_msg(corner_4, 3))
        rospy.set_param("/tables/table" + str(table) + "/objects_cuboid", [corner_1, corner_2, corner_3, corner_4])

        padded_corner_1 = [corner_1[0] - PADDING, corner_1[1] - PADDING]
        padded_corner_2 = [corner_2[0] + PADDING, corner_2[1] - PADDING]
        padded_corner_3 = [corner_3[0] + PADDING, corner_3[1] + PADDING]
        padded_corner_4 = [corner_4[0] - PADDING, corner_4[1] + PADDING]
        persons_marker_pub.publish(create_marker_msg(padded_corner_1, 0))
        persons_marker_pub.publish(create_marker_msg(padded_corner_2, 1))
        persons_marker_pub.publish(create_marker_msg(padded_corner_3, 2))
        persons_marker_pub.publish(create_marker_msg(padded_corner_4, 3))
        rospy.set_param("/tables/table" + str(table) + "/persons_cuboid", [padded_corner_1, padded_corner_2, padded_corner_3, padded_corner_4])

        now = str(datetime.datetime.now())
        rospy.set_param("/tables/table" + str(table) + "/last_updated", now)
        rospy.loginfo("Cuboid for table %d saved to parameter server", table)

        # Dump rosparams to file
        data = {
            'tables': rosparam.get_param('/tables')
        }

        with open(FILENAME, 'w') as file:
            yaml.dump(data, file)

        return TableNumberResponse(True)
    else:
        rospy.logerr("No pose data available to generate cuboid for table %d, please check if the marker is on the table", table)
        return TableNumberResponse(False)

def get_latest_pose(msg):
    global latest_pose
    latest_pose = msg

if __name__ == "__main__":

    rospy.init_node("generate_table_cuboid")
    sub = rospy.Subscriber("/aruco_single/pose", PoseStamped, get_latest_pose)
    s = rospy.Service("generate_table_cuboid", TableNumber, generate_table_cuboid)
    rospy.loginfo("Cuboid Generator Service Ready")
    
    while not rospy.is_shutdown():
        rospy.sleep(1.0)