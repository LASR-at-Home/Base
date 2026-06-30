from threading import Thread

import rclpy
from rclpy.node import Node

import yasmin
import yasmin_ros
from yasmin_viewer import YasminViewerPub

from lasr_skills import Say, GoToLocation

from doing_laundry.states import (
    Start,
    ScanShelves,
    FindAndGoToTable,
    DetectObjects,
    SelectAndVisualiseObject,
    ClassifyCategory,
    ChooseShelf,
    InstructPick,
    InstructPlace,
)

try:
    from rclpy.executors import EventsExecutor as Executor
except ImportError:
    from rclpy.executors import MultiThreadedExecutor as Executor


class DoingLaundry(yasmin.StateMachine):
    """
    Main state machine for the Pick and Place task.

    Physical manipulation is delegated to a human operator
    via verbal instructions — the robot perceives, reasons, and speaks.

    Flow:
        START
            → SCAN_SHELVES          (build shelf category map)
            → FIND_AND_GO_TO_TABLE  (locate and navigate to table)
            → DETECT_OBJECTS        (detect all objects on table)
            → SELECT_OBJECT         (pick first object, visualise for referee)
            → CLASSIFY_CATEGORY     (determine object category)
            → CHOOSE_SHELF          (match object to correct shelf)
            → INSTRUCT_PICK         (tell operator to pick up object)
            → GO_TO_CABINET         (navigate to cabinet)
            → INSTRUCT_PLACE        (tell operator which shelf to place on)
            → GO_TO_TABLE           (navigate back to table)
            → DETECT_OBJECTS        (re-scan, loop until table empty)
        → succeeded
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)

        # ── Entry ─────────────────────────────────────────────────────────────
        self.add_state(
            "START",
            Start(),
            transitions={
                "succeeded": "SCAN_SHELVES",
                "failed": "failed",
            },
        )

        # ── Scan cabinet shelves (done once) ──────────────────────────────────
        self.add_state(
            "SCAN_SHELVES",
            ScanShelves(),
            transitions={
                "succeeded": "FIND_AND_GO_TO_TABLE",
                "failed": "failed",
            },
        )

        # ── Find and navigate to table ────────────────────────────────────────
        self.add_state(
            "FIND_AND_GO_TO_TABLE",
            FindAndGoToTable(),
            transitions={
                "succeeded": "DETECT_OBJECTS",
                "failed": "DETECT_OBJECTS",  # proceed even if table not found
            },
        )

        # ── Detect all objects on table ───────────────────────────────────────
        # Re-entered at the top of every loop iteration
        self.add_state(
            "DETECT_OBJECTS",
            DetectObjects(),
            transitions={
                "succeeded": "SELECT_OBJECT",
                "failed": "DETECT_OBJECTS",  # retry until objects found
            },
        )

        # ── Select object and visualise for referee ───────────────────────────
        self.add_state(
            "SELECT_OBJECT",
            SelectAndVisualiseObject(),
            transitions={
                "succeeded": "CLASSIFY_CATEGORY",
                "failed": "DETECT_OBJECTS",  # re-scan if nothing to select
            },
        )

        # ── Classify selected object into a category ──────────────────────────
        self.add_state(
            "CLASSIFY_CATEGORY",
            ClassifyCategory(task="object"),
            transitions={
                "succeeded": "CHOOSE_SHELF",
                "failed": "CHOOSE_SHELF",  # proceed with unknown category
                "empty": "DETECT_OBJECTS",  # nothing to classify, re-scan
            },
        )

        # ── Choose which shelf to place object on ─────────────────────────────
        self.add_state(
            "CHOOSE_SHELF",
            ChooseShelf(),
            transitions={
                "succeeded": "INSTRUCT_PICK",
                "failed": "DETECT_OBJECTS",
            },
        )

        # ── Instruct operator to pick up object ───────────────────────────────
        self.add_state(
            "INSTRUCT_PICK",
            InstructPick(),
            transitions={
                "succeeded": "GO_TO_CABINET",
                "failed": "INSTRUCT_PICK",  # retry instruction
            },
        )

        # ── Navigate to cabinet ───────────────────────────────────────────────
        self.add_state(
            "GO_TO_CABINET",
            GoToLocation(location_param="doing_laundry.cabinet.pose"),
            transitions={
                "succeeded": "INSTRUCT_PLACE",
                "failed": "GO_TO_CABINET",  # retry navigation
            },
        )

        # ── Instruct operator where to place object ───────────────────────────
        self.add_state(
            "INSTRUCT_PLACE",
            InstructPlace(),
            transitions={
                "succeeded": "GO_TO_TABLE",
                "failed": "INSTRUCT_PLACE",  # retry instruction
            },
        )

        # ── Navigate back to table for next object ────────────────────────────
        self.add_state(
            "GO_TO_TABLE",
            GoToLocation(location_param="doing_laundry.table.pose"),
            transitions={
                "succeeded": "DETECT_OBJECTS",  # loop back for next object
                "failed": "GO_TO_TABLE",  # retry navigation
            },
        )


class DoingLaundryNode(Node):
    def __init__(self):
        super().__init__(
            node_name="doing_laundry",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self._executor = Executor()
        self._executor.add_node(self)
        self._spin_thread = Thread(target=self._executor.spin)
        self._spin_thread.start()


def main():
    rclpy.init()

    node = DoingLaundryNode()
    yasmin_ros.set_ros_loggers(node)

    sm = DoingLaundry()

    # Uncomment to visualise the state machine in RViz/browser
    # YasminViewerPub(sm)

    bb = yasmin.Blackboard()

    # Initialise all blackboard keys used across the machine
    bb["detected_objects"] = []
    bb["selected_object"] = None
    bb["selected_object_name"] = ""
    bb["object_name"] = ""
    bb["object_category"] = ""
    bb["shelf_data"] = {}
    bb["chosen_shelf"] = ""
    bb["chosen_shelf_str"] = ""
    bb["table_pose"] = None
    bb["debug_images"] = []

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
