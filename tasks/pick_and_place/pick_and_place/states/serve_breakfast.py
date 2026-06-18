import yasmin
import yasmin_ros

from lasr_skills import GoToLocation, Say

from pick_and_place.states.detect_bowl_spoon import DetectBowlSpoon
from pick_and_place.states.detect_cereal_milk import DetectCerealMilk
from pick_and_place.states.select_object_by_name import SelectObjectByName
from pick_and_place.states.instruct_pick import InstructPick


class ServeBreakfast(yasmin.StateMachine):
    """
    Sets up breakfast on the dining table after table cleanup is complete.

    Bowl and spoon are detected on a designated surface, cereal and milk
    are detected in the cabinet next to their respective categories.
    Every pick and place is delegated to the human operator -- detection
    is used purely for recognition scoring and referee visualisation,
    not for any manipulation. Placement instructions use fixed text via
    InstructPlaceText since breakfast rules are rule-based, not
    shelf-derived like the cleanup-phase InstructPlace.

    Sequence:
        GO_TO_BREAKFAST_SURFACE
        -> DETECT_BOWL_SPOON
        -> SELECT_BOWL -> INSTRUCT_PICK_BOWL
        -> SELECT_SPOON -> INSTRUCT_PICK_SPOON
        -> GO_TO_TABLE
        -> INSTRUCT_PLACE_BOWL    (centre of table)
        -> INSTRUCT_PLACE_SPOON   (next to bowl)
        -> GO_TO_CABINET
        -> DETECT_CEREAL_MILK
        -> SELECT_CEREAL -> INSTRUCT_PICK_CEREAL
        -> SELECT_MILK -> INSTRUCT_PICK_MILK
        -> GO_TO_TABLE
        -> INSTRUCT_PLACE_CEREAL  (next to bowl, with clearance)
        -> INSTRUCT_PLACE_MILK    (next to cereal, with clearance)
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)

        self.add_state(
            "GO_TO_BREAKFAST_SURFACE",
            GoToLocation(location_param="breakfast_surface.pose"),
            transitions={
                "succeeded": "DETECT_BOWL_SPOON",
                "failed":    "GO_TO_BREAKFAST_SURFACE",
            },
        )

        self.add_state(
            "DETECT_BOWL_SPOON",
            DetectBowlSpoon(),
            transitions={
                "succeeded": "SELECT_BOWL",
                "failed":    "DETECT_BOWL_SPOON",
            },
        )

        self.add_state(
            "SELECT_BOWL",
            SelectObjectByName(target_name="bowl"),
            transitions={
                "succeeded": "INSTRUCT_PICK_BOWL",
                "failed":    "DETECT_BOWL_SPOON",
            },
        )

        self.add_state(
            "INSTRUCT_PICK_BOWL",
            InstructPick(),
            transitions={
                "succeeded": "SELECT_SPOON",
                "failed":    "INSTRUCT_PICK_BOWL",
            },
        )

        self.add_state(
            "SELECT_SPOON",
            SelectObjectByName(target_name="spoon"),
            transitions={
                "succeeded": "INSTRUCT_PICK_SPOON",
                "failed":    "DETECT_BOWL_SPOON",
            },
        )

        self.add_state(
            "INSTRUCT_PICK_SPOON",
            InstructPick(),
            transitions={
                "succeeded": "GO_TO_TABLE_1",
                "failed":    "INSTRUCT_PICK_SPOON",
            },
        )

        self.add_state(
            "GO_TO_TABLE_1",
            GoToLocation(location_param="table.pose"),
            transitions={
                "succeeded": "INSTRUCT_PLACE_BOWL",
                "failed":    "GO_TO_TABLE_1",
            },
        )

        self.add_state(
            "INSTRUCT_PLACE_BOWL",
            Say(text="Please place the bowl in the centre of the table."),
            transitions={
                "succeeded": "INSTRUCT_PLACE_SPOON",
                "failed":    "INSTRUCT_PLACE_SPOON",
                "canceled":   "INSTRUCT_PLACE_SPOON",
            },
        )

        self.add_state(
            "INSTRUCT_PLACE_SPOON",
            Say(text="Please place the spoon next to the bowl."),
            transitions={
                "succeeded": "GO_TO_CABINET",
                "failed":    "GO_TO_CABINET",
                "canceled":   "GO_TO_CABINET",
            },
        )

        self.add_state(
            "GO_TO_CABINET",
            GoToLocation(location_param="cabinet.pose"),
            transitions={
                "succeeded": "DETECT_CEREAL_MILK",
                "failed":    "GO_TO_CABINET",
            },
        )

        self.add_state(
            "DETECT_CEREAL_MILK",
            DetectCerealMilk(),
            transitions={
                "succeeded": "SELECT_CEREAL",
                "failed":    "DETECT_CEREAL_MILK",
            },
        )

        self.add_state(
            "SELECT_CEREAL",
            SelectObjectByName(target_name="cereal"),
            transitions={
                "succeeded": "INSTRUCT_PICK_CEREAL",
                "failed":    "DETECT_CEREAL_MILK",
            },
        )

        self.add_state(
            "INSTRUCT_PICK_CEREAL",
            InstructPick(),
            transitions={
                "succeeded": "SELECT_MILK",
                "failed":    "INSTRUCT_PICK_CEREAL",
            },
        )

        self.add_state(
            "SELECT_MILK",
            SelectObjectByName(target_name="milk"),
            transitions={
                "succeeded": "INSTRUCT_PICK_MILK",
                "failed":    "DETECT_CEREAL_MILK",
            },
        )

        self.add_state(
            "INSTRUCT_PICK_MILK",
            InstructPick(),
            transitions={
                "succeeded": "GO_TO_TABLE_2",
                "failed":    "INSTRUCT_PICK_MILK",
            },
        )

        self.add_state(
            "GO_TO_TABLE_2",
            GoToLocation(location_param="table.pose"),
            transitions={
                "succeeded": "INSTRUCT_PLACE_CEREAL",
                "failed":    "GO_TO_TABLE_2",
            },
        )

        self.add_state(
            "INSTRUCT_PLACE_CEREAL",
            Say(
                text="Please place the cereal next to the bowl, "
                     "leaving at least five centimetres of clear space."
            ),
            transitions={
                "succeeded": "INSTRUCT_PLACE_MILK",
                "failed":    "INSTRUCT_PLACE_MILK",
                "canceled":   "INSTRUCT_PLACE_MILK",
            },
        )

        self.add_state(
            "INSTRUCT_PLACE_MILK",
            Say(
                text="Please place the milk next to the cereal, "
                     "leaving at least five centimetres of clear space."
            ),
            transitions={
                "succeeded": "succeeded",
                "failed":    "succeeded",
                "canceled":   "succeeded",
            },
        )