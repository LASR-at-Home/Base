import smach

from lasr_skills import Say, CheckDoorStatus
from storing_groceries.states import *

# TODO: Add recovery states

class WaitDoorOpen(smach.StateMachine):
    def __init__(self):
        super().__init__(
            outcomes=["succeeded","failed",],
            input_keys=[],
        )

        with self:
            self.go_to_table(self)

            smach.StateMachine.add(
                "CHECK_DOOR_STATUS",
                CheckDoorStatus(
                    expected_closed_depth=1.2,  # adjust for cabinet (~0.5) or room door (~1.2)
                    change_thresh=0.4,
                    open_thresh=0.6
                ),
                transitions={
                    "open": "DOOR_OPEN",
                    "closed": "CHECK_DOOR_STATUS",
                    "error": "CHECK_DOOR_STATUS",
                }
            )

            smach.StateMachine.add(
                "DOOR_OPEN",
                Say(text="Door is open"),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "aborted",
                    "preempted": "preempted",
                },            
            )

