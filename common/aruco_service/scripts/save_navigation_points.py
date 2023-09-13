#!/usr/bin/env python

import yaml
import rospy
from aruco_service.srv import TableNumber, TableNumberResponse
from geometry_msgs.msg import PoseWithCovarianceStamped
import rosparam
import rospkg

r = rospkg.RosPack()
FILENAME = r.get_path('aruco_service') + "/test_check_table_sim.yaml"

def save_points(number):
    table = number.table

    position = latest_pose.pose.pose.position
    orientation = latest_pose.pose.pose.orientation

    if table >= 0:
        rospy.set_param("/tables/table" + str(table) + "/location/position", {"x": position.x, "y": position.y, "z": 0.})
        rospy.set_param("/tables/table" + str(table) + "/location/orientation", {"x": orientation.x, "y": orientation.y, "z": orientation.z, "w": orientation.w})
        rospy.loginfo("Navigation point for table %d saved to parameter server", table)

    elif table == -1:
        rospy.set_param("/counter/location/position", {"x": position.x, "y": position.y, "z": 0.})
        rospy.set_param("counter/location/orientation",  {"x": orientation.x, "y": orientation.y, "z": orientation.z, "w": orientation.w})
        rospy.loginfo("Navigation point for the counter saved to parameter server")

    elif table == -2:
        rospy.set_param("/wait/location/position", {"x": position.x, "y": position.y, "z": 0.})
        rospy.set_param("wait/location/orientation",  {"x": orientation.x, "y": orientation.y, "z": orientation.z, "w": orientation.w})
        rospy.loginfo("Navigation point for the waiting area saved to parameter server")

    else:
        rospy.loginfo("Invalid table number.")
        return TableNumberResponse(False)
    
# Dump rosparams to file
    data = {
        'tables': rosparam.get_param('/tables'),
        'counter': rosparam.get_param('/counter'),
        'wait': rosparam.get_param('/wait')
    }

    with open(FILENAME, 'w') as file:
        yaml.dump(data, file)

    return TableNumberResponse(True)

def get_latest_pose(msg):
    global latest_pose
    latest_pose = msg

if __name__ == "__main__":

    rospy.init_node("save_navigation_points")
    sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, get_latest_pose)
    s = rospy.Service("save_navigation_points", TableNumber, save_points)

    els = rosparam.load_file(FILENAME)
    for param, ns in els:
        rosparam.upload_params(ns, param)
    
    rospy.loginfo("Navigation Points Saver Ready")
    
    while not rospy.is_shutdown():
        rospy.sleep(1.0)