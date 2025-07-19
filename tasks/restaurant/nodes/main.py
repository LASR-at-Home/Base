#!/usr/bin/env python3

import rospy
from restaurant.state_machine import Restaurant
import subprocess

if __name__ == "__main__":
    rospy.init_node("restaurant")
    restaurant = Restaurant()
    restaurant.execute()
