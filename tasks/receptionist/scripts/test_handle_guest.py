#!/usr/bin/env python3
import rospy

import smach

from receptionist.states import HandleGuest
from receptionist.states import stringify_guest_data

rospy.init_node("test_handle_guest")
sm = smach.StateMachine(outcomes=["succeeded", "failed"])
with sm:

    sm.userdata.guest_data = {
        "host": {"name": "John", "drink": "beer", "detection": False},
        "guest1": {"name": "", "drink": "", "detection": False},
        "guest2": {"name": "", "drink": "", "detection": False},
    }
    sm.add(
        "HANDLE_GUEST",
        HandleGuest("guest1", True),
        transitions={"succeeded": "succeeded", "failed": "failed"},
    )
sm.execute()

guest_data = sm.userdata.guest_data
result = stringify_guest_data(guest_data, "guest1", True)
print(result)
