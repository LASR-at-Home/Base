#!/usr/bin/env python3

import rospy
import smach
from receptionist.states import SeatGuest
from shapely.geometry import Polygon


if __name__ == "__main__":
    rospy.init_node("test_seat_guest")

    sofa_area_param = rospy.get_param("/sofa_area")

    seat_area_param = rospy.get_param("/seat_area")

    max_people_on_sofa = rospy.get_param("/max_people_on_sofa")

    rospy.sleep(5)

    sofa_area = Polygon(sofa_area_param)

    seat_area = Polygon(seat_area_param)

    seat_area = seat_area.difference(sofa_area)

    sm = smach.StateMachine(outcomes=["succeeded", "failed"])
    with sm:
        smach.StateMachine.add(
            "SeatGuest",
            SeatGuest(seat_area, sofa_area, max_people_on_sofa),
            transitions={"succeeded": "succeeded", "failed": "failed"},
        )

    outcome = sm.execute()
    rospy.loginfo("Outcome: %s", outcome)
    rospy.signal_shutdown("Test completed.")
