#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point
from restaurant.state_machine import Restaurant

if __name__ == "__main__":
    rospy.init_node("restaurant")
    location = rospy.get_param("/restaurant/bar_pose")
    menu_items = rospy.get_param("/restaurant/menu")
    bar_pose_map = Pose(
        position=Point(
            **location["pose"]["position"],
            orientation=Quaternion(**location["pose"]["orientation"])
        )
    )
    restaurant = Restaurant(bar_pose_map, menu_items)
    restaurant.execute()
