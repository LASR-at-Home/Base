import smach

from lasr_skills import Say
from storing_groceries.states import *

class WaitDoorOpen(smach.StateMachine):
    def __init__(self):
        super().__init__(
            outcomes=["succeeded","failed",],
            input_keys=[],
        )

        with self:
            self.go_to_table(self)

            smach.StateMachine.add(
                "DETECT_DOOR",
                Say(text="Detect door is on going"),
                transitions={
                    "succeeded": "DOOR_OPEN",
                    "aborted": "DOOR_OPEN",
                    "preempted": "DOOR_OPEN",
                },
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