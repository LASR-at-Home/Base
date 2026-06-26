import yasmin
import yasmin_ros
from lasr_skills import GoToLocation, Say
from pick_and_place.states.detect_objects import DetectObjects
from pick_and_place.states.select_and_visualize_object import SelectAndVisualiseObject
from pick_and_place.states.instruct_pick import InstructPick


class ServeBreakfast(yasmin.StateMachine):
    """
    Sets up breakfast on the dining table after table cleanup is complete.

    Each item is detected and instructed individually so that if one
    item fails to be detected, only that item's detection is retried —
    not the whole group.

    Sequence:
        GO_TO_BREAKFAST_SURFACE
        -> DETECT_BOWL -> SELECT_BOWL -> INSTRUCT_PICK_BOWL
        -> DETECT_SPOON -> SELECT_SPOON -> INSTRUCT_PICK_SPOON
        -> GO_TO_TABLE
        -> INSTRUCT_PLACE_BOWL
        -> INSTRUCT_PLACE_SPOON
        -> GO_TO_CABINET
        -> DETECT_CEREAL -> SELECT_CEREAL -> INSTRUCT_PICK_CEREAL
        -> DETECT_MILK -> SELECT_MILK -> INSTRUCT_PICK_MILK
        -> GO_TO_TABLE
        -> INSTRUCT_PLACE_CEREAL
        -> INSTRUCT_PLACE_MILK
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)

        # Navigate to breakfast surface
        self.add_state(
            "GO_TO_BREAKFAST_SURFACE",
            GoToLocation(location_param="pick_and_place.breakfast_surface.pose"),
            transitions={
                "succeeded": "DETECT_BOWL",
                "failed": "GO_TO_BREAKFAST_SURFACE",
            },
        )

        # Bowl
        self.add_state(
            "DETECT_BOWL",
            DetectObjects(queries=["bowl"]),
            transitions={
                "succeeded": "SELECT_BOWL",
                "failed": "DETECT_BOWL",
            },
        )

        self.add_state(
            "SELECT_BOWL",
            SelectAndVisualiseObject(target_name="bowl"),
            transitions={
                "succeeded": "INSTRUCT_PICK_BOWL",
                "finished": "DETECT_BOWL",  # not found, retry detection
            },
        )

        self.add_state(
            "INSTRUCT_PICK_BOWL",
            InstructPick(),
            transitions={
                "succeeded": "DETECT_SPOON",
                "failed": "INSTRUCT_PICK_BOWL",
            },
        )

        # Spoon
        self.add_state(
            "DETECT_SPOON",
            DetectObjects(queries=["spoon"]),
            transitions={
                "succeeded": "SELECT_SPOON",
                "failed": "DETECT_SPOON",
            },
        )

        self.add_state(
            "SELECT_SPOON",
            SelectAndVisualiseObject(target_name="spoon"),
            transitions={
                "succeeded": "INSTRUCT_PICK_SPOON",
                "finished": "DETECT_SPOON",  # not found, retry detection
            },
        )

        self.add_state(
            "INSTRUCT_PICK_SPOON",
            InstructPick(),
            transitions={
                "succeeded": "GO_TO_TABLE_1",
                "failed": "INSTRUCT_PICK_SPOON",
            },
        )

        # Navigate to table, place bowl and spoon
        self.add_state(
            "GO_TO_TABLE_1",
            GoToLocation(location_param="pick_and_place.table.pose"),
            transitions={
                "succeeded": "INSTRUCT_PLACE_BOWL",
                "failed": "GO_TO_TABLE_1",
            },
        )

        self.add_state(
            "INSTRUCT_PLACE_BOWL",
            Say(text="Please place the bowl in the centre of the table."),
            transitions={
                "succeeded": "INSTRUCT_PLACE_SPOON",
                "aborted": "INSTRUCT_PLACE_SPOON",
                "canceled": "INSTRUCT_PLACE_SPOON",
            },
        )
        self.add_state(
            "INSTRUCT_PLACE_SPOON",
            Say(text="Please place the spoon next to the bowl."),
            transitions={
                "succeeded": "GO_TO_CABINET",
                "aborted": "GO_TO_CABINET",
                "canceled": "GO_TO_CABINET",
            },
        )

        # Navigate to cabinet
        self.add_state(
            "GO_TO_CABINET",
            GoToLocation(location_param="pick_and_place.cabinet.pose"),
            transitions={
                "succeeded": "DETECT_CEREAL",
                "failed": "GO_TO_CABINET",
            },
        )

        # Cereal
        self.add_state(
            "DETECT_CEREAL",
            DetectObjects(queries=["cereal"]),
            transitions={
                "succeeded": "SELECT_CEREAL",
                "failed": "DETECT_CEREAL",
            },
        )
        self.add_state(
            "SELECT_CEREAL",
            SelectAndVisualiseObject(target_name="cereal"),
            transitions={
                "succeeded": "INSTRUCT_PICK_CEREAL",
                "finished": "DETECT_CEREAL",  # not found, retry detection
            },
        )
        self.add_state(
            "INSTRUCT_PICK_CEREAL",
            InstructPick(),
            transitions={
                "succeeded": "DETECT_MILK",
                "failed": "INSTRUCT_PICK_CEREAL",
            },
        )

        # Milk
        self.add_state(
            "DETECT_MILK",
            DetectObjects(queries=["milk"]),
            transitions={
                "succeeded": "SELECT_MILK",
                "failed": "DETECT_MILK",
            },
        )
        self.add_state(
            "SELECT_MILK",
            SelectAndVisualiseObject(target_name="milk"),
            transitions={
                "succeeded": "INSTRUCT_PICK_MILK",
                "finished": "DETECT_MILK",  # not found, retry detection
            },
        )
        self.add_state(
            "INSTRUCT_PICK_MILK",
            InstructPick(),
            transitions={
                "succeeded": "GO_TO_TABLE_2",
                "failed": "INSTRUCT_PICK_MILK",
            },
        )

        # Navigate to table, place cereal and milk
        self.add_state(
            "GO_TO_TABLE_2",
            GoToLocation(location_param="pick_and_place.table.pose"),
            transitions={
                "succeeded": "INSTRUCT_PLACE_CEREAL",
                "failed": "GO_TO_TABLE_2",
            },
        )

        self.add_state(
            "INSTRUCT_PLACE_CEREAL",
            Say(
                text="Please place the cereal next to the bowl, "
                "with sufficient space between them."
            ),
            transitions={
                "succeeded": "INSTRUCT_PLACE_MILK",
                "aborted": "INSTRUCT_PLACE_MILK",
                "canceled": "INSTRUCT_PLACE_MILK",
            },
        )
        self.add_state(
            "INSTRUCT_PLACE_MILK",
            Say(
                text="Please place the milk next to the cereal, "
                "with sufficient space between them."
            ),
            transitions={
                "succeeded": "succeeded",
                "aborted": "succeeded",
                "canceled": "succeeded",
            },
        )
