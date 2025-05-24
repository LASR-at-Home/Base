from typing import List, Optional

import smach
import smach_ros
from geometry_msgs.msg import Pose
from lasr_skills import AskAndListen, GoToLocation, Rotate, Say, Wait, HandoverObject
from restaurant.states import Survey
from std_msgs.msg import Empty
from .llm_utils import llm_inference
from std_srvs.srv import Trigger


class Restaurant(smach.StateMachine):

    def __init__(
        self,
        menu: List[str],
        bar_pose_map: Optional[Pose],
        unmapped: bool = False,
        manipulation: bool = False,
        table_pose: Optional[Pose] = None,
    ) -> None:
        super().__init__(outcomes=["succeeded", "failed"])

        if manipulation:
            raise NotImplementedError

        with self:
            smach.StateMachine.add(
                "WAIT_START",
                smach_ros.MonitorState(
                    "/restaurant/start",
                    Empty,
                    lambda _ud, _msg: False,
                ),
                transitions={
                    "valid": "WAIT_START",
                    "preempted": "WAIT_START",
                    "invalid": "SAY_START",
                },
            )

            smach.StateMachine.add(
                "SAY_START",
                Say(text="I am going to the Bar."),
                transitions={
                    "succeeded": "GO_TO_BAR_1" if not unmapped else "ROTATE_360",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            if not unmapped:

                smach.StateMachine.add(
                    "GO_TO_BAR_1",
                    GoToLocation(location=bar_pose_map),
                    transitions={"succeeded": "ROTATE_SURVEY", "failed": "failed"},
                )

            else:

                smach.StateMachine.add(
                    "ROTATE_360",
                    Rotate(angle=360.0),
                    transitions={"succeeded": "ROTATE_SURVEY"},
                )

            smach.StateMachine.add(
                "ROTATE_SURVEY",
                Rotate(angle=180.0),
                transitions={"succeeded": "SAY_WAVE"},
            )

            smach.StateMachine.add(
                "SAY_WAVE",
                Say(text="Please wave to get my attention."),
                transitions={
                    "succeeded": "SURVEY" if not unmapped else "ROTATE_360",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            if unmapped:
                smach.StateMachine.add(
                    "ROTATE_360", Rotate(angle=360), transitions={"succeeded": "SURVEY"}
                )

            smach.StateMachine.add(
                "SURVEY",
                Survey(table_pose),
                transitions={
                    "customer_found": "GO_TO_CUSTOMER",
                    "customer_not_found": "SURVEY",
                },
            )

            smach.StateMachine.add(
                "GO_TO_CUSTOMER",
                GoToLocation(),
                remapping={"location": "customer_approach_pose"},
                transitions={"succeeded": "TAKE_ORDER", "failed": "failed"},
            )

            smach.StateMachine.add(
                "TAKE_ORDER",
                AskAndListen(
                    tts_phrase="Hello, I'm TIAGo. What can I get for you today?"
                ),
                remapping={"transcribed_speech": "order_str"},
                transitions={
                    "succeeded": "PROCESS_ORDER",
                    "failed": "TAKE_ORDER_AGAIN",
                },
            )

            @smach.cb_interface(
                input_keys=["order_str"],
                output_keys=["order_str"],
                outcomes=["succeeded", "failed"],
            )
            def speech_postprocess_cb(ud):
                try:
                    response = llm_inference.restaurant_llm(ud.order_str, menu)

                    def menu_to_string(order_dict):
                        parts = []
                        print(order_dict)

                        for items_str in order_dict.values():
                            if items_str == "None":
                                continue
                            items = [
                                item.strip()
                                for item in items_str.split(",")
                                if item.strip()
                            ]
                            if not items:
                                continue
                            elif len(items) == 1:
                                parts.append(f"a {items[0]}")
                            else:
                                formatted_items = (
                                    ", ".join(items[:-1]) + f", and {items[-1]}"
                                )
                                parts.append(formatted_items)

                        if not parts:
                            return ""
                        elif len(parts) == 1:
                            return f"Please get me {parts[0]}."
                        else:
                            return f"Please get me {', '.join(parts[:-1])}, and {parts[-1]}."

                    order_str = menu_to_string(response)

                    if not order_str:
                        return "failed"

                    ud.order_str = order_str
                except Exception as e:
                    print(e)
                    return "failed"
                return "succeeded"

            smach.StateMachine.add(
                "PROCESS_ORDER",
                smach.CBState(speech_postprocess_cb),
                transitions={"succeeded": "SAY_ORDER", "failed": "TAKE_ORDER_AGAIN"},
            )

            smach.StateMachine.add(
                "TAKE_ORDER_AGAIN",
                AskAndListen(
                    tts_phrase="Sorry, I didn't understand you. What would you like to order?",
                ),
                remapping={"transcribed_speech": "order_str"},
                transitions={"succeeded": "PROCESS_ORDER", "failed": "failed"},
            )

            smach.StateMachine.add(
                "SAY_ORDER",
                Say(text="Coming right up!"),
                transitions={
                    "succeeded": "GO_TO_BAR",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "GO_TO_BAR",
                GoToLocation(location=bar_pose_map),
                transitions={"succeeded": "REQUEST_ITEMS", "failed": "failed"},
            )

            if not manipulation:
                smach.StateMachine.add(
                    "REQUEST_ITEMS",
                    Say(format_str="{} I will turn around for you to load the order."),
                    remapping={"placeholders": "order_str"},
                    transitions={
                        "succeeded": "ROTATE_LOAD",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )

                smach.StateMachine.add(
                    "ROTATE_LOAD",
                    Rotate(angle=180.0),
                    transitions={"succeeded": "WAIT_LOAD"},
                )

                smach.StateMachine.add(
                    "WAIT_LOAD",
                    Wait(5),
                    transitions={"succeeded": "RETURN_TO_CUSTOMER", "failed": "failed"},
                )
            else:
                smach.StateMachine.add(
                    "REQUEST_ITEMS",
                    Say(format_str="{} Please place the item in front of me."),
                    remapping={"placeholders": "order_str"},
                    transitions={
                        "succeeded": "WAIT_PREPARE",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )
                smach.StateMachine.add(
                    "WAIT_PREPARE",
                    Wait(5),
                    transitions={"succeeded": "GRASP_OBJECT", "failed": "failed"},
                )
                smach.StateMachine.add(
                    "GRASP_OBJECT",
                    smach_ros.ServiceState(
                        "tiago_kcl_moveit_grasp/grasp_object", Trigger
                    ),
                    transitions={
                        "succeeded": "RETURN_TO_CUSTOMER",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )

            smach.StateMachine.add(
                "RETURN_TO_CUSTOMER",
                GoToLocation(),
                remapping={"location": "customer_approach_pose"},
                transitions={"succeeded": "SAY_TAKE_ORDER", "failed": "failed"},
            )

            if not manipulation:

                smach.StateMachine.add(
                    "SAY_TAKE_ORDER",
                    Say(
                        text="Here is your order, I will turn around for you to unload it."
                    ),
                    transitions={
                        "succeeded": "ROTATE_UNLOAD",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )

                smach.StateMachine.add(
                    "ROTATE_UNLOAD",
                    Rotate(angle=180.0),
                    transitions={"succeeded": "WAIT_UNLOAD"},
                )

                smach.StateMachine.add(
                    "WAIT_UNLOAD",
                    Wait(5),
                    transitions={"succeeded": "GO_TO_SURVEY", "failed": "failed"},
                )

            else:
                smach.StateMachine.add(
                    "HANDOVER",
                    HandoverObject("order"),
                    transitions={"succeeded": "GO_TO_SURVEY", "failed": "failed"},
                )

            smach.StateMachine.add(
                "GO_TO_SURVEY",
                GoToLocation(location=bar_pose_map),
                transitions={"succeeded": "ROTATE_SURVEY", "failed": "failed"},
            )
