#!/usr/bin/env python3

import smach
import rospy

from lasr_skills import AdjustCamera

if __name__ == "__main__":
    rospy.init_node("test_adjust_camera")
    sm = smach.StateMachine(outcomes=["end"], output_keys=[])
    with sm:
        sm.add(
            "AdjustCamera",
            AdjustCamera(
                max_attempts=1000,
                debug=True,
                init_state="u1m",
            ),
            transitions={"finished": "end", "failed": "end", "truncated": "end"},
        )
    sm.execute()
    rospy.signal_shutdown("down")
