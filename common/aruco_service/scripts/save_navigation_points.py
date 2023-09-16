#!/usr/bin/env python

import yaml
import rospy
from aruco_service.srv import SaveNavigationPoint, SaveNavigationPointResponse
from geometry_msgs.msg import PoseWithCovarianceStamped
import rosparam

def save_navigation_points(number):
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
        return SaveNavigationPointResponse(False)
    
# Dump rosparams to file
    data = {
        'tables': rosparam.get_param('/tables'),
        'counter': rosparam.get_param('/counter'),
        'wait': rosparam.get_param('/wait')
    }

    with open(rosparam.get_param("/config_path"), 'w') as file:
        yaml.dump(data, file)

    return SaveNavigationPointResponse(True)

def save_pre_navigation_points(number):
    table = number.table

    position = latest_pose.pose.pose.position
    orientation = latest_pose.pose.pose.orientation

    if table >= 0:
        rospy.set_param("/tables/table" + str(table) + "/pre_location/position", {"x": position.x, "y": position.y, "z": 0.})
        rospy.set_param("/tables/table" + str(table) + "/pre_location/orientation", {"x": orientation.x, "y": orientation.y, "z": orientation.z, "w": orientation.w})
        rospy.loginfo("Pre-navigation point for table %d saved to parameter server", table)

    else:
        rospy.loginfo("Invalid table number.")
        return SaveNavigationPointResponse(False)
    
    # Dump rosparams to file
    data = {
        'tables': rosparam.get_param('/tables'),
        'counter': rosparam.get_param('/counter'),
        'wait': rosparam.get_param('/wait')
    }

    with open(rosparam.get_param("/config_path"), 'w') as file:
        yaml.dump(data, file)

    return SaveNavigationPointResponse(True)

def get_latest_pose(msg):
    global latest_pose
    latest_pose = msg

if __name__ == "__main__":

    rospy.init_node("save_navigation_points")
    sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, get_latest_pose)
    rospy.Service("save_navigation_points", SaveNavigationPoint, save_navigation_points)
    rospy.Service("save_pre_navigation_points", SaveNavigationPoint, save_pre_navigation_points)    
    rospy.loginfo("Navigation Points Saver Ready")
    
    while not rospy.is_shutdown():
        rospy.sleep(1.0)