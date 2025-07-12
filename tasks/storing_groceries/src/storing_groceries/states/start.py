import smach
import smach_ros

from std_msgs.msg import Empty

from lasr_skills import DetectDoorOpening, Say


class Start(smach.StateMachine):

    def __init__(self):
        super().__init__(outcomes=["succeeded"])

        with self:
            smach.StateMachine.add(
                "WAIT_START",
                smach_ros.MonitorState(
                    "/storing_groceries/start",
                    Empty,
                    lambda *_: False,
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
                    "succeeded": "SAY_WAITING",
                    "aborted": "SAY_WAITING",
                    "preempted": "SAY_WAITING",
                },
            )

            smach.StateMachine.add(
                "SAY_WAITING",
                Say(text="Waiting for the door to open."),
                transitions={
                    "succeeded": "WAIT_FOR_DOOR_TO_OPEN",
                    "aborted": "WAIT_FOR_DOOR_TO_OPEN",
                    "preempted": "WAIT_FOR_DOOR_TO_OPEN",
                },
            )

            smach.StateMachine.add(
                "WAIT_FOR_DOOR_TO_OPEN",
                DetectDoorOpening(timeout=10.0),
                transitions={
                    "door_opened": "succeeded",
                },
            )
