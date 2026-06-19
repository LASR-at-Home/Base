from threading import Thread

import rclpy
from rclpy.node import Node

import yasmin
import yasmin_ros
from yasmin_viewer import YasminViewerPub

from lasr_skills import Say, GoToLocation

from pick_and_place.states import (
    Start,
    DetectObjects,
    SelectAndVisualiseObject,
    ClassifyCategory,
    DecideDestination,
    ChooseShelf,
    InstructPick,
    InstructPlace,
    AddTableCollision,  
    GraspObject, 
    ApproachTable
)

from rclpy.executors import MultiThreadedExecutor as Executor

class PickAndPlace(yasmin.StateMachine):
    """
    Main state machine for the Pick and Place task (announce-only).

    Physical manipulation is delegated to a human operator via verbal
    instructions — the robot perceives, reasons, routes, and speaks.

    Each detected table object is routed to one of THREE destinations
    (the task-planning core of the challenge):
        - tableware / cutlery  → dishwasher
        - the trash category   → trash bin
        - everything else      → cabinet (matched to a shelf)

    Flow:
        START                       (start signal, door, drive to table)
            → DETECT_OBJECTS        (open-vocab detect all table objects, ONCE)
            ┌→ SELECT_OBJECT        (pop next object; empty → FINISH)
            │   → CLASSIFY_CATEGORY (determine object category)
            │   → DECIDE_DESTINATION(dishwasher / trash bin / cabinet)
            │        ├ cabinet → CHOOSE_SHELF
            │        └ other   ─────────────┐
            │   → INSTRUCT_PICK  ←───────────┘
            │   → GO_TO_DESTINATION (drive to chosen destination pose)
            │   → INSTRUCT_PLACE
            │   → GO_TO_TABLE       (drive back to table)
            └───(loop)
            → FINISH                (announce completion)
        → succeeded

    NOTE: ScanShelves (perceiving the cabinet shelves and announcing their
    categories) is added in the next iteration; until then shelf_data is {}
    and ChooseShelf uses its category-name fallback.
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)

        # ── Entry ─────────────────────────────────────────────────────────────
        self.add_state(
            "START",
            Start(),
            transitions={
                "succeeded": "ADD_TABLE_COLLISION",
                "failed":    "failed",
            },
        )

        self.add_state(
            "ADD_TABLE_COLLISION",
            AddTableCollision(head_tilt=-0.6),     # детект усього столу здалеку
            transitions={
                "succeeded": "GO_TO_TABLE_FOR_PICK",   # було "DETECT_OBJECTS"
            },
        )

        self.add_state(
            "GO_TO_TABLE_FOR_PICK",
            GoToLocation(location_param="pick_and_place.table.pose"),
            transitions={
                "succeeded": "DETECT_OBJECTS",
                "failed":    "DETECT_OBJECTS",   # все одно пробуємо детект
            },
        )

        # ── Detect all objects on the table (done ONCE) ───────────────────────
        self.add_state(
            "DETECT_OBJECTS",
            DetectObjects(),
            transitions={
                "succeeded": "SELECT_OBJECT",
                "failed":    "DETECT_OBJECTS",  # retry until objects found
            },
        )

        # ── Select next object and visualise for referee ──────────────────────
        self.add_state(
            "SELECT_OBJECT",
            SelectAndVisualiseObject(),
            transitions={
                "succeeded": "CLASSIFY_CATEGORY",
                "finished":  "FINISH",          # all objects processed
            },
        )

        # ── Classify selected object into a category ──────────────────────────
        self.add_state(
            "CLASSIFY_CATEGORY",
            ClassifyCategory(task="object"),
            transitions={
                "succeeded": "DECIDE_DESTINATION",
                "failed":    "DECIDE_DESTINATION",  # proceed with unknown category
                "empty":     "SELECT_OBJECT",       # nothing to classify, next object
            },
        )

        # ── Decide destination: dishwasher / trash bin / cabinet ──────────────
        self.add_state(
            "DECIDE_DESTINATION",
            DecideDestination(),
            transitions={
                "cabinet": "CHOOSE_SHELF",
                "other":   "INSTRUCT_PICK",
            },
        )

        # ── Choose which cabinet shelf to place object on ─────────────────────
        self.add_state(
            "CHOOSE_SHELF",
            ChooseShelf(),
            transitions={
                "succeeded": "INSTRUCT_PICK",
                "failed":    "INSTRUCT_PICK",  # announce anyway
            },
        )

        # ── Instruct operator to pick up object ───────────────────────────────
        self.add_state(
            "INSTRUCT_PICK",
            InstructPick(),
            transitions={
                "succeeded": "GRASP",
                "failed":    "INSTRUCT_PICK",  # retry instruction
            },
        )
        self.add_state(
            "GRASP", GraspObject(),
            transitions={"succeeded": "GO_TO_DESTINATION", "failed": "GO_TO_DESTINATION"},
        )
        # ── Navigate to the chosen destination (pose set by DecideDestination)─
        self.add_state(
            "GO_TO_DESTINATION",
            GoToLocation(),  # reads blackboard["location"]
            transitions={
                "succeeded": "INSTRUCT_PLACE",
                "failed":    "INSTRUCT_PLACE",  # announce even if nav failed
            },
        )

        # ── Instruct operator where to place object ───────────────────────────
        self.add_state(
            "INSTRUCT_PLACE",
            InstructPlace(),
            transitions={
                "succeeded": "GO_TO_TABLE",
                "failed":    "INSTRUCT_PLACE",  # retry instruction
            },
        )

        # ── Navigate back to table for next object ────────────────────────────
        self.add_state(
            "GO_TO_TABLE",
            GoToLocation(location_param="pick_and_place.table.pose"),
            transitions={
                "succeeded": "SELECT_OBJECT",  # loop back for next object
                "failed":    "GO_TO_TABLE",     # retry navigation
            },
        )

        # ── Done ──────────────────────────────────────────────────────────────
        self.add_state(
            "FINISH",
            Say(
                text="I have sorted all the objects I could see on the table. "
                     "Pick and place complete."
            ),
            transitions={
                "succeeded": "succeeded",
                "aborted":   "succeeded",
                "canceled":  "succeeded",
            },
        )

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__(
            node_name="pick_and_place",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self._executor = Executor()
        self._executor.add_node(self)
        self._spin_thread = Thread(target=self._executor.spin)
        self._spin_thread.start()


def main():
    rclpy.init()

    node = PickAndPlaceNode()
    yasmin_ros.set_ros_loggers(node)

    sm = PickAndPlace()
  
    # Uncomment to visualise the state machine in RViz/browser
    # YasminViewerPub(sm)

    bb = yasmin.Blackboard()

    # Initialise all blackboard keys used across the machine
    bb["detected_objects"]     = []
    bb["selected_object"]      = None
    bb["selected_object_name"] = ""
    bb["object_name"]          = ""
    bb["object_category"]      = ""
    bb["shelf_data"]           = {}
    bb["chosen_shelf"]         = ""
    bb["chosen_shelf_str"]     = ""
    bb["destination"]          = ""
    bb["destination_str"]      = ""
    bb["location"]             = None
    bb["table_pose"]           = None
    bb["debug_images"]         = []

    try:
        outcome = sm(bb)
        yasmin.YASMIN_LOG_INFO(f"Pick and Place finished with outcome: {outcome}")
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(str(e))

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()