#!/usr/bin/env python3
import rospy
from receptionist.sm_new import Receptionist

if __name__ == "__main__":
    rospy.init_node("receptionist_robocup")
    receptionist = Receptionist()
    outcome = receptionist.execute()
    print(outcome)
    rospy.spin()

    