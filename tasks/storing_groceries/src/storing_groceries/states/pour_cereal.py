import smach

from lasr_skills import Say, AdjustCamera, GoToLocation, DetectDict
from storing_groceries.states import *


class PourCereal(smach.StateMachine):
    def __init__(self, table_pose):
        super().__init__(
            outcomes=[
                "succeeded",
                "failed",
            ],
            input_keys=[],
        )

        self.table_pose = table_pose
        self.userdata.cereal = []
        self.userdata.not_graspable = [
            "tv",
            "television",
            "chair",
            "table",
            "couch",
            "person",
            "dog",
            "cat",
            "monitor",
            "computer",
            "fan",
            "microwave",
            "stove",
            "refrigerator",
            "cabinet",
            "door",
            "laptop",
            "printer",
            "keyboard",
            "plant",
            "sink",
            "bed",
            "sofa",
            "board",
        ]

        with self:
            self.go_to_table()

            smach.StateMachine.add(
                "SAY_DETECT_CEREAL",
                Say("Start detecting table for cereal and container"),
                transitions={
                    "succeeded": "DETECT_CEREAL",
                    "aborted": "DETECT_CEREAL",
                    "preempted": "DETECT_CEREAL",
                },
            )

            smach.StateMachine.add(
                "DETECT_CEREAL",
                DetectDict(),
                transitions={
                    "succeeded": "CHOOSE_CEREAL",
                    "failed": "CHOOSE_CEREAL",
                },
                remapping={"detections": "table_objects"},
            )

            smach.StateMachine.add(
                "CHOOSE_CEREAL",
                ChooseObject("cereal_only"),
                transitions={
                    "succeeded": "SAY_DETECTED_CEREAL",
                    "failed": "SAY_NOT_DETECTED_CEREAL",
                    "empty": "SAY_NOT_DETECTED_CEREAL",  # should return escape in future
                },
            )

            smach.StateMachine.add(
                "SAY_DETECTED_CEREAL",
                Say(text="Detected cereal"),
                transitions={
                    "succeeded": "GRAB_OBJECT",
                    "aborted": "GRAB_OBJECT",
                    "preempted": "GRAB_OBJECT",
                },
            )

            smach.StateMachine.add(
                "SAY_NOT_DETECTED_CEREAL",
                Say(text="Couldn't find a cereal. I will end this now."),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "succeeded",
                    "preempted": "succeeded",
                },
            )  # recovery befaviour needed

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
        """Adds the states to go to table area."""

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
                "succeeded": f"SAY_DETECT_CEREAL",
                "aborted": f"SAY_DETECT_CEREAL",
                "preempted": f"SAY_DETECT_CEREAL",
            },
        )
