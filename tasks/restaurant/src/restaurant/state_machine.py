import smach
import smach_ros
from geometry_msgs.msg import Pose
from lasr_skills import AskAndListen, GoToLocation, Rotate, Say, Wait
from restaurant.speech.speech_handlers import handle_speech
from restaurant.states import Survey
from std_msgs.msg import Empty


class Restaurant(smach.StateMachine):

    def __init__(self, bar_pose_map: Pose) -> None:
        super().__init__(outcomes=["succeeded", "failed"])

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
                Say(text="Start of Restaurant task."),
                transitions={
                    "succeeded": "SURVEY",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "SURVEY",
                Survey(),
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
                transitions={"succeeded": "PROCESS_ORDER", "failed": "failed"},
            )

            @smach.cb_interface(
                input_keys=["order_str"],
                output_keys=["order_str"],
                outcomes=["succeeded", "failed"],
            )
            def speech_postprocess_cb(ud):
                parsed_order = handle_speech(ud.order_str, False)
                print(parsed_order)
                order_string = ", ".join(
                    [
                        f"{count} {item if count == 1 or item.endswith('s') else item+'s'}"
                        for count, item in parsed_order
                    ]
                )
                ud.order_str = order_string
                print(order_string)
                return "succeeded"

            smach.StateMachine.add(
                "PROCESS_ORDER",
                smach.CBState(speech_postprocess_cb),
                transitions={"succeeded": "SAY_ORDER", "failed": "failed"},
            )

            smach.StateMachine.add(
                "SAY_ORDER",
                Say(format_str="Your order is: {}. I will deliver it shortly."),
                remapping={"placeholders": "order_str"},
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

            smach.StateMachine.add(
                "REQUEST_ITEMS",
                Say(
                    format_str="Please get me {}, I will turn around for you to load the order."
                ),
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
                transitions={"succeeded": "WAIT_LOAD", "failed": "failed"},
            )

            smach.StateMachine.add(
                "WAIT_LOAD",
                Wait(5),
                transitions={"succeeded": "RETURN_TO_CUSTOMER", "failed": "failed"},
            )

            smach.StateMachine.add(
                "RETURN_TO_CUSTOMER",
                GoToLocation(),
                remapping={"location": "customer_approach_pose"},
                transitions={"succeeded": "SAY_TAKE_ORDER", "failed": "failed"},
            )

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
                transitions={"succeeded": "WAIT_UNLOAD", "failed": "failed"},
            )

            smach.StateMachine.add(
                "WAIT_UNLOAD",
                Wait(5),
                transitions={"succeeded": "GO_TO_SURVEY", "failed": "failed"},
            )

            smach.StateMachine.add(
                "GO_TO_SURVEY",
                GoToLocation(location=bar_pose_map),
                transitions={"succeeded": "SURVEY", "failed": "failed"},
            )
