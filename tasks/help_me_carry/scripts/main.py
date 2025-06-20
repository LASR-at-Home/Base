#!/usr/bin/env python3

import rospy
from carry_my_luggage.state_machine import CarryMyLuggage

if __name__ == "__main__":
    rospy.init_node("carry_my_luggage_robocup")
    carry_my_luggage = CarryMyLuggage()
    outcome = carry_my_luggage.execute()
    rospy.loginfo(f"Carry my luggage finished with outcome: {outcome}")
    rospy.spin()
