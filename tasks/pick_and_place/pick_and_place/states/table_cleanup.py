import yasmin
import yasmin_ros

from lasr_skills import Say, GoToLocation

from pick_and_place.states.detect_objects import DetectObjects
from pick_and_place.states.select_and_visualize_object import SelectAndVisualiseObject
from pick_and_place.states.classify_category import ClassifyCategory
from pick_and_place.states.decide_destination import DecideDestination
from pick_and_place.states.choose_shelf import ChooseShelf
from pick_and_place.states.instruct_pick import InstructPick
from pick_and_place.states.instruct_place import InstructPlace
from pick_and_place.states.detect_floor_trash import DetectFloorTrash


class TableCleanup(yasmin.StateMachine):
    """
    Cleans the dining table: detects all objects once, then loops through
    each one — classify, decide destination (dishwasher / trash bin /
    cabinet), choose shelf if cabinet-bound, instruct pick, navigate,
    instruct place, navigate back for the next object.

    After the table is clear, also checks the floor near the trash bin
    for the optional floor trash item (rulebook +30 bonus).

    Sequence:
        GO_TO_TABLE_FOR_PICK
        -> DETECT_OBJECTS               (done once)
        ┌-> SELECT_OBJECT               (empty -> SAY_CLEANUP_DONE)
        │     -> CLASSIFY_CATEGORY
        │     -> DECIDE_DESTINATION
        │          ├ cabinet -> CHOOSE_SHELF
        │          └ other   ───────────┐
        │     -> INSTRUCT_PICK  <───────┘
        │     -> GO_TO_DESTINATION
        │     -> INSTRUCT_PLACE
        │     -> GO_TO_TABLE
        └─────(loop)
        -> SAY_CLEANUP_DONE
        -> GO_TO_TRASH_BIN_FLOOR
        -> DETECT_FLOOR_TRASH           (optional, skips if nothing found)
        -> SELECT_FLOOR_TRASH -> INSTRUCT_PICK_FLOOR -> INSTRUCT_PLACE_FLOOR
        -> succeeded
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)

        # Navigate to table
        self.add_state(
            "GO_TO_TABLE_FOR_PICK",
            GoToLocation(location_param="pick_and_place.table.pose"),
            transitions={
                "succeeded": "DETECT_OBJECTS",
                "failed":    "DETECT_OBJECTS",
            },
        )

        # Detect all objects on the table (done ONCE)
        self.add_state(
            "DETECT_OBJECTS",
            DetectObjects(),
            transitions={
                "succeeded": "SELECT_OBJECT",
                "failed":    "DETECT_OBJECTS",
            },
        )

        # Select next object and visualise for referee
        self.add_state(
            "SELECT_OBJECT",
            SelectAndVisualiseObject(),
            transitions={
                "succeeded": "CLASSIFY_CATEGORY",
                "finished":  "SAY_CLEANUP_DONE",
            },
        )

        # Classify selected object into a category
        self.add_state(
            "CLASSIFY_CATEGORY",
            ClassifyCategory(task="object"),
            transitions={
                "succeeded": "DECIDE_DESTINATION",
                "failed":    "DECIDE_DESTINATION",
                "empty":     "SELECT_OBJECT",
            },
        )

        # Decide destination: dishwasher / trash bin / cabinet
        self.add_state(
            "DECIDE_DESTINATION",
            DecideDestination(),
            transitions={
                "cabinet": "CHOOSE_SHELF",
                "other":   "INSTRUCT_PICK",
            },
        )

        # Choose which cabinet shelf to place object on
        self.add_state(
            "CHOOSE_SHELF",
            ChooseShelf(),
            transitions={
                "succeeded": "INSTRUCT_PICK",
                "failed":    "INSTRUCT_PICK",
            },
        )

        # Instruct operator to pick up object
        self.add_state(
            "INSTRUCT_PICK",
            InstructPick(),
            transitions={
                "succeeded": "GO_TO_DESTINATION",
                "failed":    "INSTRUCT_PICK",
            },
        )

        # Navigate to the chosen destination (pose set by DecideDestination)
        self.add_state(
            "GO_TO_DESTINATION",
            GoToLocation(),  # reads blackboard["location"]
            transitions={
                "succeeded": "INSTRUCT_PLACE",
                "failed":    "INSTRUCT_PLACE",
            },
        )

        # Instruct operator where to place object
        self.add_state(
            "INSTRUCT_PLACE",
            InstructPlace(),
            transitions={
                "succeeded": "GO_TO_TABLE",
                "failed":    "INSTRUCT_PLACE",
            },
        )

        # Navigate back to table for next object
        self.add_state(
            "GO_TO_TABLE",
            GoToLocation(location_param="pick_and_place.table.pose"),
            transitions={
                "succeeded": "SELECT_OBJECT",
                "failed":    "GO_TO_TABLE",
            },
        )

        # Table done, check the floor near the trash bin
        self.add_state(
            "SAY_CLEANUP_DONE",
            Say(
                text="I have finished cleaning the table. "
                     "Let me check the floor near the trash bin."
            ),
            transitions={
                "succeeded": "GO_TO_TRASH_BIN_FLOOR",
                "aborted":   "GO_TO_TRASH_BIN_FLOOR",
                "canceled":  "GO_TO_TRASH_BIN_FLOOR",
            },
        )

        self.add_state(
            "GO_TO_TRASH_BIN_FLOOR",
            GoToLocation(location_param="pick_and_place.trash_bin.pose"),
            transitions={
                "succeeded": "DETECT_FLOOR_TRASH",
                "failed":    "DETECT_FLOOR_TRASH",
            },
        )

        self.add_state(
            "DETECT_FLOOR_TRASH",
            DetectFloorTrash(),
            transitions={
                "succeeded": "SELECT_FLOOR_TRASH",
                "failed":    "succeeded",   # nothing found, floor trash optional
            },
        )

        self.add_state(
            "SELECT_FLOOR_TRASH",
            SelectAndVisualiseObject(),
            transitions={
                "succeeded": "INSTRUCT_PICK_FLOOR",
                "finished":  "succeeded",
            },
        )

        self.add_state(
            "INSTRUCT_PICK_FLOOR",
            InstructPick(),
            transitions={
                "succeeded": "INSTRUCT_PLACE_FLOOR",
                "failed":    "INSTRUCT_PICK_FLOOR",
            },
        )

        self.add_state(
            "INSTRUCT_PLACE_FLOOR",
            Say(text="Please place it in the trash bin."),
            transitions={
                "succeeded": "succeeded",
                "aborted":   "succeeded",
                "canceled":  "succeeded",
            },
        )