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

class PickAndPlaceCleanUpTable(yasmin.StateMachine):

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
            "GO_TO_EXTRA_SURFACE_FOR_PICK",
            GoToLocation(location_param="pick_and_place.extra_surface.pose"),
            transitions={
                "succeeded": "DETECT_OBJECTS",
                "failed":    "DETECT_OBJECTS",
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
                "succeeded": "CHOOSE_SHELF",
                "failed":    "CHOOSE_SHELF",  # proceed with unknown category
                "empty":     "SELECT_OBJECT",       # nothing to classify, next object
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

        # ── Navigate to the chosen destination (pose set by DecideDestination)─
        self.add_state(
            "GO_TO_DESTINATION",
            GoToLocation(location_param="pick_and_place.table.cabinet"),
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
            "GO_TO_EXTRA_SURFACE",
            GoToLocation(location_param="pick_and_place.extra_surface.pose"),
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