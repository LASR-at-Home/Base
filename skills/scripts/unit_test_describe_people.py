#!/usr/bin/env python3

import smach
import rospy

from lasr_skills import DescribePeople

if __name__ == "__main__":
    rospy.init_node("test_describe")

    sm = smach.StateMachine(outcomes=["end"], output_keys=["people"])

    with sm:
        sm.add(
            "DESCRIBE",
            DescribePeople(),
            transitions={"succeeded": "end", "failed": "end"},
        )

    sm.execute()

    print("\n\nDetected people:", sm.userdata["people"][0]["features"])

    rospy.signal_shutdown("down")
