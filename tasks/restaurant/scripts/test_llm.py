#!/usr/bin/env python3

import rospy

import smach

from restaurant.states import HandleOrder

if __name__ == "__main__":
    rospy.init_node("restaurant_test_llm")

    sm = smach.StateMachine(outcomes=["succeeded", "failed"])

    sm.userdata.customer_transcription = (
        "ummm.. get me one coffee, a big coke. I'll also have a fanta."
    )

    with sm:
        smach.StateMachine.add(
            "HANDLE_ORDER",
            HandleOrder(),
            transitions={"succeeded": "PRINT_ORDER", "failed": "failed"},
        )

        def _print_order(userdata):
            print(userdata.order)
            print(userdata.order_str)
            return "succeeded"

        smach.StateMachine.add(
            "PRINT_ORDER",
            smach.CBState(
                _print_order,
                input_keys=["order", "order_str"],
                outcomes=[
                    "succeeded",
                ],
            ),
            transitions={"succeeded": "succeeded"},
        )

    sm.execute()
