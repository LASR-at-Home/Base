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
from pick_and_place.states.detect_trash_floor import DetectFloorTrash
from pick_and_place.states.scan_shelves_if_needed import ScanShelvesIfNeeded


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
                "failed": "DETECT_OBJECTS",
            },
        )

        # Detect all objects on the table (done ONCE)
        self.add_state(
            "DETECT_OBJECTS",
            DetectObjects(location_param="table", model="best.pt"),
            transitions={
                "succeeded": "SELECT_OBJECT",
                "failed": "DETECT_OBJECTS",
            },
        )

        # Select next object and visualise for referee
        self.add_state(
            "SELECT_OBJECT",
            SelectAndVisualiseObject(),
            transitions={
                "succeeded": "CLASSIFY_CATEGORY",
                "finished": "CLOSE_DISHWASHER_IF_OPENED",  # ← change from SAY_CLEANUP_DONE
            },
        )

        # Close dishwasher if it was opened
        self.add_state(
            "CLOSE_DISHWASHER_IF_OPENED",
            CloseDishwasherIfOpened(),
            transitions={
                "succeeded": "SAY_CLEANUP_DONE",
                "skipped": "SAY_CLEANUP_DONE",
            },
        )

        # Classify selected object into a category
        self.add_state(
            "CLASSIFY_CATEGORY",
            ClassifyCategory(task="object"),
            transitions={
                "succeeded": "DECIDE_DESTINATION",
                "failed": "DECIDE_DESTINATION",
                "empty": "SELECT_OBJECT",
            },
        )

        # Decide destination: dishwasher / trash bin / cabinet
        self.add_state(
            "DECIDE_DESTINATION",
            DecideDestination(),
            transitions={
                "cabinet": "CHOOSE_SHELF",
                "other": "INSTRUCT_PICK",
            },
        )

        # Choose which cabinet shelf to place object on
        self.add_state(
            "CHOOSE_SHELF",
            ChooseShelf(),
            transitions={
                "succeeded": "INSTRUCT_PICK",
                "failed": "INSTRUCT_PICK",
            },
        )

        # Instruct operator to pick up object
        self.add_state(
            "INSTRUCT_PICK",
            InstructPick(),
            transitions={
                "succeeded": "GO_TO_DESTINATION",
                "failed": "INSTRUCT_PICK",
            },
        )

        # Navigate to the chosen destination
        self.add_state(
            "GO_TO_DESTINATION",
            GoToLocation(),  # reads blackboard["location"]
            transitions={
                "succeeded": "OPEN_DISHWASHER_IF_NEEDED",
                "failed": "OPEN_DISHWASHER_IF_NEEDED",
            },
        )

        # Open dishwasher on first dish visit
        self.add_state(
            "OPEN_DISHWASHER_IF_NEEDED",
            OpenDishwasherIfNeeded(),
            transitions={
                "succeeded": "SCAN_SHELVES_IF_NEEDED",
                "skipped": "SCAN_SHELVES_IF_NEEDED",
            },
        )

        # Scan shelves on first cabinet visit
        self.add_state(
            "SCAN_SHELVES_IF_NEEDED",
            ScanShelvesIfNeeded(),
            transitions={
                "succeeded": "INSTRUCT_PLACE",
                "skipped": "INSTRUCT_PLACE",
            },
        )

        # Instruct operator where to place object
        self.add_state(
            "INSTRUCT_PLACE",
            InstructPlace(),
            transitions={
                "succeeded": "GO_TO_TABLE",
                "failed": "INSTRUCT_PLACE",
            },
        )

        # Navigate back to table for next object
        self.add_state(
            "GO_TO_TABLE",
            GoToLocation(location_param="pick_and_place.table.pose"),
            transitions={
                "succeeded": "SELECT_OBJECT",
                "failed": "GO_TO_TABLE",
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
                "aborted": "GO_TO_TRASH_BIN_FLOOR",
                "canceled": "GO_TO_TRASH_BIN_FLOOR",
            },
        )

        self.add_state(
            "GO_TO_TRASH_BIN_FLOOR",
            GoToLocation(location_param="pick_and_place.trash_bin.pose"),
            transitions={
                "succeeded": "DETECT_FLOOR_TRASH",
                "failed": "DETECT_FLOOR_TRASH",
            },
        )

        self.add_state(
            "DETECT_FLOOR_TRASH",
            DetectFloorTrash(),
            transitions={
                "succeeded": "SET_FLOOR_TRASH_CONTEXT",
                "failed": "succeeded",  # nothing found, floor trash optional
            },
        )

        self.add_state(
            "SELECT_FLOOR_TRASH",
            SelectAndVisualiseObject(),
            transitions={
                "succeeded": "INSTRUCT_PICK_FLOOR",
                "finished": "succeeded",
            },
        )

        self.add_state(
            "SET_FLOOR_TRASH_CONTEXT",
            yasmin.CbState(
                outcomes=["succeeded"],
                callback=lambda bb: [
                    bb.__setitem__("object_category", "trash"),
                    bb.__setitem__("destination_str", "the trash bin"),
                    bb.__setitem__("chosen_shelf", ""),
                    bb.__setitem__("chosen_shelf_str", ""),
                ]
                and "succeeded",
            ),
            transitions={"succeeded": "INSTRUCT_PICK_FLOOR"},
        )

        self.add_state(
            "INSTRUCT_PICK_FLOOR",
            InstructPick(),
            transitions={
                "succeeded": "INSTRUCT_PLACE_FLOOR",
                "failed": "INSTRUCT_PICK_FLOOR",
            },
        )

        self.add_state(
            "INSTRUCT_PLACE_FLOOR",
            Say(text="Please place it in the trash bin."),
            transitions={
                "succeeded": "succeeded",
                "aborted": "succeeded",
                "canceled": "succeeded",
            },
        )


class OpenDishwasherIfNeeded(yasmin.State):
    """Says open dishwasher only on first dish item visit."""

    def __init__(self):
        super().__init__(outcomes=["succeeded", "skipped"])
        self.add_input_key("destination")
        self.add_input_key("dishwasher_opened")
        self.add_output_key("dishwasher_opened")

    def execute(self, blackboard) -> str:
        if blackboard["destination"] != "dishwasher":
            return "skipped"
        if blackboard["dishwasher_opened"]:
            return "skipped"
        say = Say(text="Please open the dishwasher.")
        say.execute(blackboard)
        blackboard["dishwasher_opened"] = True
        return "succeeded"


class CloseDishwasherIfOpened(yasmin.State):
    def __init__(self):
        super().__init__(outcomes=["succeeded", "skipped"])
        self.add_input_key("dishwasher_opened")

    def execute(self, blackboard) -> str:
        if not blackboard["dishwasher_opened"]:
            return "skipped"
        say = Say(text="Please close the dishwasher.")
        say.execute(blackboard)
        return "succeeded"
