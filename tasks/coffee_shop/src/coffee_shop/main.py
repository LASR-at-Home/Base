#!/usr/bin/env python3
import rospy
from coffee_shop.phases import Phase1, Phase2, Phase3

if __name__ == "__main__":
    rospy.init_node("test")
    rospy.spin()
