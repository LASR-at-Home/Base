#!/usr/bin/env python3
import rospy
from storing_groceries.state_machine import StoringGroceries

if __name__ == "__main__":
    rospy.init_node("storing_groceries")
    sm = StoringGroceries()
    sm.execute()
