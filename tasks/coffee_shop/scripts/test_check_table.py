#!/usr/bin/env python3
import smach
from coffee_shop.phases.phase_1.states import CheckTable
from coffee_shop.context import Context
import rospy
import sys

if __name__ == "__main__":
    rospy.init_node("test_check_table")
    sm = smach.StateMachine(outcomes=["end", "not_finished"])
    context = Context(sys.argv[1], sys.argv[2])
    context.current_table = "table1"

    with sm:
        sm.add(
            "CHECK_TABLE",
            CheckTable(context),
            transitions={
                "has_needs_serving_tables": "end",
                "not_finished": "end",
                "idle": "end",
                "has_free_tables": "end",
            },
        )
    sm.execute()
    rospy.signal_shutdown("down")
