#!/usr/bin/env python3
import rospy
from lab_restaurant.state_machine import LabRestaurant#

import smach
import smach_ros

from geometry_msgs.msg import Pose, Point, Quaternion, Polygon, PolygonStamped
from shapely.geometry import Polygon as ShapelyPolygon

from std_msgs.msg import Header


if __name__ == "__main__":
    rospy.init_node("lab_restaurant_node")

    loc_A_param = rospy.get_param("/lab_restaurant/loc_A")

    loc_A = Pose(
        position=Point(**loc_A_param["position"]),
        orientation=Quaternion(**loc_A_param["orientation"]),
    )


    loc_B_param = rospy.get_param("/lab_restaurant/loc_B")

    loc_B = Pose(
        position=Point(**loc_B_param["position"]),
        orientation=Quaternion(**loc_B_param["orientation"]),
    )

    labRestaurant = LabRestaurant(
        loc_A,
        loc_B,
    )

    outcome = labRestaurant.execute()

    # rospy.loginfo(f"Restaurant finished with outcome: {outcome}")
    rospy.spin()



