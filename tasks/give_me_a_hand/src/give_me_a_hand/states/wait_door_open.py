import smach

from lasr_skills import Say, CheckDoorStatus
from give_me_a_hand.states import *

# TODO: Add recovery states and prevent infinit loop


class WaitDoorOpen(smach.StateMachine):
    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "aborted", "preempted"],
            input_keys=[],
        )

        with self:

            smach.StateMachine.add(
                "CHECK_DOOR_STATUS",
                CheckDoorStatus(
                    expected_closed_depth=1.2,  # adjust for cabinet (~0.5) or room door (~1.2)
                    change_thresh=0.4,
                    open_thresh=0.6,
                ),
                transitions={
                    "open": "DOOR_OPEN",
                    "closed": "CHECK_DOOR_STATUS",
                    "error": "CHECK_DOOR_STATUS",
                },
            )

            smach.StateMachine.add(
                "DOOR_OPEN",
                Say(text="Door is open, Starting give me a hand"),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "aborted",
                    "preempted": "preempted",
                },
            )
