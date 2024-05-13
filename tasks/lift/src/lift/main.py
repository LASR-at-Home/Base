#!/usr/bin/env python3
import rospy
from lift.sm import Lift

if __name__ == "__main__":
    rospy.init_node("take_lift_sciroc")
    lift = Lift()
    outcome = lift.execute()
    print(outcome)
    rospy.spin()
