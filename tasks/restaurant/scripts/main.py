#!/usr/bin/env python3

import rospy
import rosservice
from geometry_msgs.msg import PoseWithCovarianceStamped
from pal_navigation_msgs.msg import NavigationStatus
from restaurant.state_machine import Restaurant

if __name__ == "__main__":
    rospy.init_node("restaurant")
    bar_pose_map = rospy.wait_for_message("/robot_pose", PoseWithCovarianceStamped)
    unmapped = (
        rospy.wait_for_message("/pal_navigation_sm/state", NavigationStatus).status.data
        != "LOC"
    )
    restaurant = Restaurant(
        bar_pose_map.pose.pose,
        unmapped,
        False,
    )
    restaurant.execute()
