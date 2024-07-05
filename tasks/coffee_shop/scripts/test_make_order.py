#!/usr/bin/env python3
import smach
import rospy
from coffee_shop.phases.phase_2.states import (
    MakeOrder,
    CheckOrder,
    LoadOrder,
    WaitForOrder,
    GoToCounter,
)
from coffee_shop.context import Context
import sys

if __name__ == "__main__":
    rospy.init_node("test_make_order")
    sm = smach.StateMachine(outcomes=["end"])
    context = Context(sys.argv[1], sys.argv[2])
    context.current_table = "table0"
    context.tables[context.current_table]["status"] = "currently serving"
    context.tables[context.current_table]["order"] = ["cup", "cup"]

    with sm:
        sm.add(
            "GO_TO_COUNTER", GoToCounter(context), transitions={"done": "MAKE_ORDER"}
        )
        sm.add("MAKE_ORDER", MakeOrder(context), transitions={"done": "WAIT_FOR_ORDER"})
        sm.add(
            "WAIT_FOR_ORDER", WaitForOrder(context), transitions={"done": "CHECK_ORDER"}
        )
        sm.add(
            "CHECK_ORDER",
            CheckOrder(context),
            transitions={"correct": "LOAD_ORDER", "incorrect": "WAIT_FOR_ORDER"},
        )
        sm.add("LOAD_ORDER", LoadOrder(context), transitions={"done": "end"})

    sm.execute()
    rospy.signal_shutdown("down")
