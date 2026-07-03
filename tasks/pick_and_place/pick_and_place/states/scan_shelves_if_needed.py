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
from pick_and_place.states.scan_shelves import ScanShelves


class ScanShelvesIfNeeded(yasmin.State):
    """
    Runs ScanShelves only on the first cabinet visit.
    Skips if shelf_data is already populated or destination is not cabinet.
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "skipped"])
        self.add_input_key("shelf_data")
        self.add_input_key("destination")
        self._scanner = ScanShelves()

    def execute(self, blackboard) -> str:
        # Only scan if going to cabinet
        if blackboard["destination"] != "cabinet":
            return "skipped"

        # Skip if already scanned on a previous visit
        if blackboard["shelf_data"]:
            yasmin.YASMIN_LOG_INFO("Shelf data already populated — skipping scan.")
            return "skipped"

        yasmin.YASMIN_LOG_INFO("First cabinet visit — scanning shelves.")
        outcome = self._scanner.execute(blackboard)
        return "succeeded" if outcome == "succeeded" else "skipped"
