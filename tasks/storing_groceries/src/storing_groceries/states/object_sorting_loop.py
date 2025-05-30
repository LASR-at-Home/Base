import smach

from lasr_skills import Say, AdjustCamera, GoToLocation
from storing_groceries.states import (
    GetNameAndInterest,
    ReceptionistLearnFaces,
    GetGuestAttributes,
)

class ObjectSortingLoop(smach.StateMachine):
    def __init__(self):
        super().__init__(
            outcomes=["succeeded","failed",],
            input_keys=[],
        )

        with self:
            self.go_to_table(self)

            smach.StateMachine.add(
                "DETECT_TABLE",
                Say(text="Detect table is on going"),
                transitions={
                    "succeeded": "SELECT_OBJECT",
                    "aborted": "SELECT_OBJECT",
                    "preempted": "SELECT_OBJECT",
                },
            )

            smach.StateMachine.add(
                "SELECT_OBJECT",
                Say(text="Detect table is on going"),
                transitions={
                    "succeeded": "CLASSIFY_CATEGORY_OBJECT",
                    "aborted": "CLASSIFY_CATEGORY_OBJECT",
                    "preempted": "CLASSIFY_CATEGORY_OBJECT",
                },
            )

            smach.StateMachine.add(
                "CLASSIFY_CATEGORY_OBJECT",
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
                    "succeeded": "GO_TO_CABINET",
                    "aborted": "GO_TO_CABINET",
                    "preempted": "GO_TO_CABINET",
                },
            )

            self.go_to_cabinet(self)

            smach.StateMachine.add(
                "DETECT_CABINET",
                Say(text="Detect table is on going"),
                transitions={
                    "succeeded": "CLASSIFY_CATEGORY_CABINET",
                    "aborted": "CLASSIFY_CATEGORY_CABINET",
                    "preempted": "CLASSIFY_CATEGORY_CABINET",
                },
            )

            smach.StateMachine.add(
                "CLASSIFY_CATEGORY_CABINET",
                Say(text="Detect table is on going"),
                transitions={
                    "succeeded": "PUT_OBJECT",
                    "aborted": "PUT_OBJECT",
                    "preempted": "PUT_OBJECT",
                },
            )

            smach.StateMachine.add(
                "PUT_OBJECT",
                Say(text="Detect table is on going"),
                transitions={
                    "succeeded": "GO_TO_TABLE",
                    "aborted": "GO_TO_TABLE",
                    "preempted": "GO_TO_TABLE",
                },
            )


    
    def go_to_table(self, userdata, cereal=False) -> None:
        """Adds the states to go to table area.
        """
        
        smach.StateMachine.add(
            f"GO_TO_TABLE",
            GoToLocation(userdata.table_pose),
            transitions={
                "succeeded": f"SAY_ARRIVE_TABLE",
                "failed": f"SAY_ARRIVE_TABLE",
            },
        )
        
        if(cereal):
            smach.StateMachine.add(
                f"SAY_ARRIVE_CEREAL",
                Say(text="Arrive table"),
                transitions={
                    "succeeded": f"DETECT_CEREAL",
                    "aborted": f"DETECT_CEREAL",
                    "preempted": f"DETECT_CEREAL",
                },
            )    
        else:   
            smach.StateMachine.add(
                f"SAY_ARRIVE_TABLE",
                Say(text="Arrive table"),
                transitions={
                    "succeeded": f"DETECT_TABLE",
                    "aborted": f"DETECT_TABLE",
                    "preempted": f"DETECT_TABLE",
                },
            )

    def go_to_cabinet(self, userdata) -> None:
        """Adds the states to go to cabinet area.
        """
        
        smach.StateMachine.add(
            f"GO_TO_CABINET",
            GoToLocation(userdata.table_pose),
            transitions={
                "succeeded": f"SAY_ARRIVE_CABINET",
                "failed": f"SAY_ARRIVE_CABINET",
            },
        )

        smach.StateMachine.add(
            f"SAY_ARRIVE_CABINET",
            Say(text="Arrive cabinet"),
            transitions={
                "succeeded": f"DETECT_CABINET",
                "aborted": f"DETECT_CABINET",
                "preempted": f"DETECT_CABINET",
            },
        )
