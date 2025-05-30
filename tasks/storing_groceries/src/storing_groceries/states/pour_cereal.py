import smach

from lasr_skills import Say, AdjustCamera, GoToLocation
from storing_groceries.states import (
    GetNameAndInterest,
    ReceptionistLearnFaces,
    GetGuestAttributes,
)

class PourCereal(smach.StateMachine):
    def __init__(self):
        super().__init__(
            outcomes=["succeeded","failed",],
            input_keys=[],
        )

        with self:
            self.go_to_table(self)

            smach.StateMachine.add(
                "DETECT_CEAREAL",
                Say(text="Detect table is on going"),
                transitions={
                    "succeeded": "SELECT_OBJECT",
                    "aborted": "SELECT_OBJECT",
                    "preempted": "SELECT_OBJECT",
                },
            )

            smach.StateMachine.add(
                "DETECT_CONTAINER",
                Say(text="Detect table is on going"),
                transitions={
                    "succeeded": "GRAB_OBJECT",
                    "aborted": "GRAB_OBJECT",
                    "preempted": "GRAB_OBJECT",
                },
            )

            smach.StateMachine.add(
                "GRAB_OBJECT",
                Say(text="Detect table is on going"),
                transitions={
                    "succeeded": "POUR_OBJECT",
                    "aborted": "POUR_OBJECT",
                    "preempted": "POUR_OBJECT",
                },
            )

            smach.StateMachine.add(
                "POUR_OBJECT",
                Say(text="Detect table is on going"),
                transitions={
                    "succeeded": "SELECT_OBJECT",
                    "aborted": "SELECT_OBJECT",
                    "preempted": "SELECT_OBJECT",
                },
            )

    def go_to_table(self, userdata, cereal=False) -> None:
        """Adds the states to go to table area.
        """
        
        smach.StateMachine.add(
            f"GO_TO_CEREAL",
            GoToLocation(userdata.table_pose),
            transitions={
                "succeeded": f"SAY_ARRIVE_TABLE",
                "failed": f"SAY_ARRIVE_TABLE",
            },
        )
        
        smach.StateMachine.add(
            f"SAY_ARRIVE_CEREAL",
            Say(text="Arrive table"),
            transitions={
                "succeeded": f"DETECT_CEREAL",
                "aborted": f"DETECT_CEREAL",
                "preempted": f"DETECT_CEREAL",
            },
        )    