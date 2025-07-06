#!/usr/bin/env python3

import rospy
from restaurant.state_machine import Restaurant
import subprocess

if __name__ == "__main__":
    rospy.init_node("restaurant")
    restaurant = Restaurant()
    restaurant.execute()
    subprocess.call(["rosservice", "call", "/pal_navigation_sm", "input: 'LOC'"])
    subprocess.call(
        [
            "rosservice",
            "call",
            "/pal_map_manager/change_map",
            "input: '2025-06-17-lab'",
        ]
    )
