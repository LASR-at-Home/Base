import smach

from lasr_skills import AskAndListen, Say, AdjustCamera
from storing_groceries.states import (
    GetNameAndInterest,
    ReceptionistLearnFaces,
    GetGuestAttributes,
)


class ObjectSortingLoop(smach.StateMachine):
    def __init__(self):
        super().__init__(
            outcomes=["succeeded","failed",],
            input_keys=["guest_data"],
        )

        self.go_to_table()

        with self:
            smach.StateMachine.add(
                "DETECT_TABLE",
                Say(text="Detect table is on going"),
                transitions={
                    "succeeded": "WAIT_DOOR_OPEN",
                    "aborted": "GO_TO_WAIT_LOCATION",
                    "preempted": "GO_TO_WAIT_LOCATION",
                },
            )

            smach.StateMachine.add(
                "SELECT_OBJECT",
                Say(text="Detect table is on going"),
                transitions={
                    "succeeded": "WAIT_DOOR_OPEN",
                    "aborted": "GO_TO_WAIT_LOCATION",
                    "preempted": "GO_TO_WAIT_LOCATION",
                },
            )

            smach.StateMachine.add(
                "CLASSIFY_CATEGORY_OBJECT",
                Say(text="Detect table is on going"),
                transitions={
                    "succeeded": "WAIT_DOOR_OPEN",
                    "aborted": "GO_TO_WAIT_LOCATION",
                    "preempted": "GO_TO_WAIT_LOCATION",
                },
            )

            smach.StateMachine.add(
                "GRAB_OBJECT",
                Say(text="Detect table is on going"),
                transitions={
                    "succeeded": "WAIT_DOOR_OPEN",
                    "aborted": "GO_TO_WAIT_LOCATION",
                    "preempted": "GO_TO_WAIT_LOCATION",
                },
            )

        self.go_to_cabinet()
    
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
                    "succeeded": f"OBJECT_SORTING_LOOP",
                    "aborted": f"OBJECT_SORTING_LOOP",
                    "preempted": f"OBJECT_SORTING_LOOP",
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
