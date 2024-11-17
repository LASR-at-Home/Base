import smach
import smach_ros
from geometry_msgs.msg import Pose
from lasr_skills import AskAndListen, GoToLocation, Rotate, Say
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
                    "succeeded": "GO_TO_SURVEY",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "ROTATE_UNLOAD",
                Rotate(angle=180.0),
                transitions={"succeeded": "GO_TO_SURVEY", "failed": "failed"},
            )

            smach.StateMachine.add(
                "GO_TO_SURVEY",
                GoToLocation(location=bar_pose_map),
                transitions={"succeeded": "SURVEY", "failed": "failed"},
            )
