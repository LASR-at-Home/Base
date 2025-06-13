import smach

from lasr_skills import Say, AdjustCamera, GoToLocation
from storing_groceries.states import *

class PourCereal(smach.StateMachine):
    def __init__(self, table_pose):
        super().__init__(
            outcomes=["succeeded","failed",],
            input_keys=[],
        )

        self.table_pose = table_pose

        with self:
            self.go_to_table()

            smach.StateMachine.add(
                "DETECT_CEREAL",
                Say(text="Detect cereal is on going"),
                transitions={
                    "succeeded": "DETECT_CONTAINER",
                    "aborted": "DETECT_CONTAINER",
                    "preempted": "DETECT_CONTAINER",
                },
            )

            smach.StateMachine.add(
                "DETECT_CONTAINER",
                Say(text="Detect container is on going"),
                transitions={
                    "succeeded": "GRAB_OBJECT",
                    "aborted": "GRAB_OBJECT",
                    "preempted": "GRAB_OBJECT",
                },
            )

            smach.StateMachine.add(
                "GRAB_OBJECT",
                Say(text="Grab object is on going"),
                transitions={
                    "succeeded": "POUR_OBJECT",
                    "aborted": "POUR_OBJECT",
                    "preempted": "POUR_OBJECT",
                },
            )

            smach.StateMachine.add(
                "POUR_OBJECT",
                Say(text="pour object is on going"),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "succeeded",
                    "preempted": "succeeded",
                },
            )

    def go_to_table(self) -> None:
        """Adds the states to go to table area.
        """
        
        smach.StateMachine.add(
            f"GO_TO_CEREAL",
            GoToLocation(self.table_pose),
            transitions={
                "succeeded": f"SAY_ARRIVE_CEREAL",
                "failed": f"SAY_ARRIVE_CEREAL",
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