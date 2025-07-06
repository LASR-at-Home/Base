import smach
import smach_ros

from lasr_skills import Say

from std_msgs.msg import Empty


class StoringGroceries(smach.StateMachine):

    def __init__(self) -> None:
        super().__init__(outcomes=["succeeded", "failed"])

        with self:
            smach.StateMachine.add(
                "WAIT_START",
                smach_ros.MonitorState(
                    "/restaurant/start",
                    Empty,
                    lambda _: False,
                ),
                transitions={
                    "valid": "WAIT_START",
                    "preempted": "WAIT_START",
                    "invalid": "SAY_START",
                },
            )

            smach.StateMachine.add(
                "SAY_START",
                Say(text="Start of Storing Groceries task."),
                transitions={
                    "succeeded": "",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )
