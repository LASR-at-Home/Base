import smach
import smach_ros

from std_msgs.msg import Empty

from lasr_skills import DetectDoorOpening, Say, GoToLocation


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
                DetectDoorOpening(timeout=1.0),
                transitions={
                    "door_opened": "SAY_GOING_TO_TABLE",
                },
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_TABLE",
                Say(text="I am going to the table"),
                transitions={
                    "succeeded": "GO_TO_TABLE",
                    "aborted": "GO_TO_TABLE",
                    "preempted": "GO_TO_TABLE",
                },
            )

            smach.StateMachine.add(
                "GO_TO_TABLE",
                GoToLocation(location_param="/storing_groceries/table/pose"),
                transitions={
                    "succeeded": "ASK_OPEN_CABINET_DOOR",
                    "failed": "ASK_OPEN_CABINET_DOOR",
                },
            )

            smach.StateMachine.add(
                "ASK_OPEN_CABINET_DOOR",
                Say(
                    text="I am unable to open either of the cabinet doors. Please open them for me."
                ),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "succeeded",
                    "preempted": "succeeded",
                },
            )
