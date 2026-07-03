import yasmin
import yasmin_ros

from lasr_skills import Say, GoToLocation

from pick_and_place.states.detect_objects import DetectObjects
from pick_and_place.states.select_and_visualize_object import SelectAndVisualiseObject
from pick_and_place.states.classify_category import ClassifyCategory
from pick_and_place.states.choose_shelf import ChooseShelf
from pick_and_place.states.instruct_pick import InstructPick
from pick_and_place.states.instruct_place import InstructPlace


class ExtraSurfaceCleanup(yasmin.StateMachine):
    """
    Clears the extra surface, which holds exactly two objects from the
    common objects set per the rulebook. Both always go to the cabinet,
    grouped by category or similarity — unlike dining table cleanup,
    there is no dishwasher/trash routing here since extra surface items
    are not tableware, cutlery, or designated trash.

    Sequence:
        SAY_GOING_TO_EXTRA_SURFACE
        -> GO_TO_EXTRA_SURFACE
        -> DETECT_OBJECTS          (done once, both objects)
        ┌-> SELECT_OBJECT          (empty -> succeeded)
        │     -> CLASSIFY_CATEGORY
        │     -> CHOOSE_SHELF
        │     -> INSTRUCT_PICK
        │     -> GO_TO_CABINET
        │     -> INSTRUCT_PLACE
        │     -> GO_TO_EXTRA_SURFACE  (loop back for second object)
        └─────(loop)
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)

        # Announce and navigate to extra surface
        self.add_state(
            "SAY_GOING_TO_EXTRA_SURFACE",
            Say(text="I am now going to check the extra surface."),
            transitions={
                "succeeded": "GO_TO_EXTRA_SURFACE",
                "aborted": "GO_TO_EXTRA_SURFACE",
                "canceled": "GO_TO_EXTRA_SURFACE",
            },
        )

        self.add_state(
            "GO_TO_EXTRA_SURFACE",
            GoToLocation(location_param="pick_and_place.extra_surface.pose"),
            transitions={
                "succeeded": "DETECT_OBJECTS",
                "failed": "DETECT_OBJECTS",
            },
        )

        # Detect both objects on the extra surface
        self.add_state(
            "DETECT_OBJECTS",
            DetectObjects(location_param="extra_surface", model="best.pt"),
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
                "finished": "succeeded",  # both objects processed
            },
        )

        # Classify selected object
        self.add_state(
            "CLASSIFY_CATEGORY",
            ClassifyCategory(task="object"),
            transitions={
                "succeeded": "CHOOSE_SHELF",
                "failed": "CHOOSE_SHELF",  # proceed with unknown category
                "empty": "SELECT_OBJECT",  # nothing to classify, next object
            },
        )

        # Always goes to the cabinet — choose which shelf
        self.add_state(
            "CHOOSE_SHELF",
            ChooseShelf(),
            transitions={
                "succeeded": "INSTRUCT_PICK",
                "failed": "INSTRUCT_PICK",  # announce anyway
            },
        )

        # Instruct operator to pick up object
        self.add_state(
            "INSTRUCT_PICK",
            InstructPick(),
            transitions={
                "succeeded": "GO_TO_CABINET",
                "failed": "INSTRUCT_PICK",
            },
        )

        # Navigate to cabinet
        self.add_state(
            "GO_TO_CABINET",
            GoToLocation(location_param="pick_and_place.cabinet.pose"),
            transitions={
                "succeeded": "INSTRUCT_PLACE",
                "failed": "INSTRUCT_PLACE",  # announce even if nav failed
            },
        )

        # Instruct operator where to place object
        self.add_state(
            "INSTRUCT_PLACE",
            InstructPlace(),
            transitions={
                "succeeded": "GO_TO_EXTRA_SURFACE_LOOP",
                "failed": "INSTRUCT_PLACE",
            },
        )

        # Navigate back to extra surface for the second object
        self.add_state(
            "GO_TO_EXTRA_SURFACE_LOOP",
            GoToLocation(location_param="pick_and_place.extra_surface.pose"),
            transitions={
                "succeeded": "SELECT_OBJECT",  # loop back for next object
                "failed": "GO_TO_EXTRA_SURFACE_LOOP",
            },
        )
