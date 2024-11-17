#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from restaurant.state_machine import Restaurant

if __name__ == "__main__":
    rospy.init_node("restaurant")
    bar_pose_map = rospy.wait_for_message("/robot_pose", PoseWithCovarianceStamped)
    restaurant = Restaurant(bar_pose_map.pose.pose)
    restaurant.execute()
