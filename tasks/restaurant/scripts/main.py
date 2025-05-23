#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point
from restaurant.state_machine import Restaurant

MANIPULATION: bool = False
HARDCODED_TABLE_POSE: bool = False

if __name__ == "__main__":
    rospy.init_node("restaurant")
    location = rospy.get_param("/restaurant/bar_pose")
    menu_items = rospy.get_param("/restaurant/menu")
    bar_pose_map = Pose(
        position=Point(**location["position"]),
        orientation=Quaternion(**location["orientation"]),
    )
    table_location = rospy.get_param("/restaurant/table_pose")
    table_pose = (
        Pose(
            position=Point(**table_location["position"]),
            orientation=Quaternion(**table_location["orientation"]),
        )
        if HARDCODED_TABLE_POSE
        else None
    )
    restaurant = Restaurant(
        bar_pose_map, menu_items, MANIPULATION, HARDCODED_TABLE_POSE
    )
    restaurant.execute()
