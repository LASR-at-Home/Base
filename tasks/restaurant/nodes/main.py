#!/usr/bin/env python3

import rospy
from restaurant.state_machine import Restaurant

if __name__ == "__main__":
    rospy.init_node("restaurant")
    restaurant = Restaurant()
    restaurant.execute()
