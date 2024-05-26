#!/usr/bin/env python3

import rospy
import smach
from receptionist.states import FindAndLookAt

if __name__ == "__main__":
    rospy.init_node("test_find_and_look_at")

    sm = FindAndLookAt(
        "nicole",
        [
            [0.0, 0.0],
            [-1.0, 0.0],
            [1.0, 0.0],
        ],
    )
    sm.userdata.guest_name = "nicole"
    sm.userdata.dataset = "receptionist"
    sm.userdata.confidence = 0.5

    outcome = sm.execute()

    rospy.loginfo(f"Outcome: {outcome}")
