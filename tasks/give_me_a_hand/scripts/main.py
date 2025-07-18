#!/usr/bin/env python3

import rospy
from give_me_a_hand.state_machine import GiveMeAHand
import smach
import smach_ros

from geometry_msgs.msg import Pose, Point, Quaternion, Polygon, PolygonStamped
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.validation import explain_validity

from std_msgs.msg import Header

if __name__ == "__main__":
    rospy.init_node("give_me_a_hand_robocup")
  
    # Launch state machine
    give_me_a_hand = GiveMeAHand()

    outcome = give_me_a_hand.execute()
    rospy.loginfo(f"Give me a hand finished with outcome: {outcome}")

    rospy.spin()
