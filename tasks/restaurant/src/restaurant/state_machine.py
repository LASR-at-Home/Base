import smach
import smach_ros
from geometry_msgs.msg import Pose
from lasr_skills import AskAndListen, GoToLocation, Rotate, Say, Wait
from restaurant.states import Survey, HandleOrder, GetPoses
from std_msgs.msg import Empty


class Restaurant(smach.StateMachine):

    def __init__(self) -> None:
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

            # To attempt to build a map
            smach.StateMachine.add(
                "ROTATE_360",
                Rotate(angle=360.0),
                transitions={"succeeded": "GET_POSES", "failed": "failed"},
            )

            smach.StateMachine.add(
                "GET_POSES", GetPoses(), transitions={"succeeded": "SURVEY"}
            )

            smach.StateMachine.add(
                "SURVEY",
                Survey((-71.0, 71.0), 10),
                transitions={
                    "customer_found": "GET_APPROACH_POSE",
                    "customer_not_found": "SURVEY",
                },
                remapping={
                    "hands_up_detections": "hands_up_detections",
                },
            )

            # smach.StateMachine.add(
            #     "GO_TO_CUSTOMER",
            #     GoToLocation(),
            #     remapping={"location": "customer_approach_pose"},
            #     transitions={"succeeded": "TAKE_ORDER", "failed": "failed"},
            # )

            # smach.StateMachine.add(
            #     "TAKE_ORDER",
            #     AskAndListen(
            #         tts_phrase="Hello, I'm TIAGo. What can I get for you today?"
            #     ),
            #     remapping={"transcribed_speech": "order_str"},
            #     transitions={"succeeded": "HANDLE_ORDER", "failed": "failed"},
            # )

            # smach.StateMachine.add(
            #     "HANDLE_ORDER",
            #     HandleOrder(),
            #     remapping={"customer_transcription": "order_str"},
            #     transitions={"succeeded": "SAY_ORDER", "failed": "failed"},
            # )

            # smach.StateMachine.add(
            #     "SAY_ORDER",
            #     Say(format_str="Your order is: {}. I will deliver it shortly."),
            #     remapping={"placeholders": "order_str"},
            #     transitions={
            #         "succeeded": "GO_TO_BAR",
            #         "preempted": "failed",
            #         "aborted": "failed",
            #     },
            # )

            # smach.StateMachine.add(
            #     "GO_TO_BAR",
            #     GoToLocation(location=bar_pose_map),
            #     transitions={"succeeded": "REQUEST_ITEMS", "failed": "failed"},
            # )

            # smach.StateMachine.add(
            #     "REQUEST_ITEMS",
            #     Say(
            #         format_str="Please get me {}, I will turn around for you to load the order."
            #     ),
            #     remapping={"placeholders": "order_str"},
            #     transitions={
            #         "succeeded": "ROTATE_LOAD",
            #         "aborted": "failed",
            #         "preempted": "failed",
            #     },
            # )

            # smach.StateMachine.add(
            #     "ROTATE_LOAD",
            #     Rotate(angle=180.0),
            #     transitions={"succeeded": "WAIT_LOAD", "failed": "failed"},
            # )

            # smach.StateMachine.add(
            #     "WAIT_LOAD",
            #     Wait(5),
            #     transitions={"succeeded": "RETURN_TO_CUSTOMER", "failed": "failed"},
            # )

            # smach.StateMachine.add(
            #     "RETURN_TO_CUSTOMER",
            #     GoToLocation(),
            #     remapping={"location": "customer_approach_pose"},
            #     transitions={"succeeded": "SAY_TAKE_ORDER", "failed": "failed"},
            # )

            # smach.StateMachine.add(
            #     "SAY_TAKE_ORDER",
            #     Say(
            #         text="Here is your order, I will turn around for you to unload it."
            #     ),
            #     transitions={
            #         "succeeded": "ROTATE_UNLOAD",
            #         "aborted": "failed",
            #         "preempted": "failed",
            #     },
            # )

            # smach.StateMachine.add(
            #     "ROTATE_UNLOAD",
            #     Rotate(angle=180.0),
            #     transitions={"succeeded": "WAIT_UNLOAD", "failed": "failed"},
            # )

            # smach.StateMachine.add(
            #     "WAIT_UNLOAD",
            #     Wait(5),
            #     transitions={"succeeded": "GO_TO_SURVEY", "failed": "failed"},
            # )

            # smach.StateMachine.add(
            #     "GO_TO_SURVEY",
            #     GoToLocation(location=bar_pose_map),
            #     transitions={"succeeded": "SURVEY", "failed": "failed"},
            # )
