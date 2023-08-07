#!/usr/bin/env python

import yaml
import rospy
import datetime
from aruco_service.srv import TableNumber, TableNumberResponse
from geometry_msgs.msg import PoseStamped
import rosparam

# Units are in meters
# The marker should be placed on the bottom left corner of the table, with the x axis pointing to the right and the y axis pointing up
TABLE_LONG_SIDE = 1.2
TABLE_SHORT_SIDE = 0.6
PADDING = 0.5
FILENAME = "/home/peter/robocup_ws/src/aruco_service/test_check_table_sim.yaml"

def generate_table_cuboid(number):
    table = number.table
    seconds = rospy.get_time()

    if latest_pose is not None and (seconds - latest_pose.header.stamp.secs < 1.0):

        corner_1 = [latest_pose.pose.position.x - 0.05, latest_pose.pose.position.y - 0.05] # subtract 5cm because of marker size to get corner
        corner_2 = [latest_pose.pose.position.x - 0.05 + TABLE_LONG_SIDE, latest_pose.pose.position.y - 0.05]
        corner_3 = [latest_pose.pose.position.x - 0.05 + TABLE_LONG_SIDE, latest_pose.pose.position.y - 0.05 + TABLE_SHORT_SIDE]
        corner_4 = [latest_pose.pose.position.x - 0.05, latest_pose.pose.position.y - 0.05 + TABLE_SHORT_SIDE]

        rospy.set_param("/tables/table" + str(table) + "/objects_cuboid", [corner_1, corner_2, corner_3, corner_4])

        padded_corner_1 = [corner_1[0] - PADDING, corner_1[1] - PADDING]
        padded_corner_2 = [corner_2[0] + PADDING, corner_2[1] - PADDING]
        padded_corner_3 = [corner_3[0] + PADDING, corner_3[1] + PADDING]
        padded_corner_4 = [corner_4[0] - PADDING, corner_4[1] + PADDING]
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