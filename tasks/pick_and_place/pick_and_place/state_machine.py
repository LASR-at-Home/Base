from threading import Thread

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

import yasmin
import yasmin_ros
from yasmin_viewer import YasminViewerPub

from lasr_skills import Say

from pick_and_place.states import (
    Start,
    TableCleanup,
    ServeBreakfast,
    ExtraSurfaceCleanup,
)

from rclpy.executors import MultiThreadedExecutor as Executor


class PickAndPlace(yasmin.StateMachine):
    """
    Top-level state machine for the Pick and Place task.

    Orchestrates three independent sub-machines in the rulebook's
    suggested order: clean the dining table, serve breakfast, then
    clean the extra surface. Each sub-machine is self-contained and
    can be tested in isolation.

    Flow:
        WAIT_START -> SAY_START -> START                  (door, drive to table)
            -> SAY_STARTING_CLEANUP -> TABLE_CLEANUP       (TableCleanup)
            -> SAY_STARTING_BREAKFAST -> SERVE_BREAKFAST   (ServeBreakfast)
            -> SAY_STARTING_EXTRA_SURFACE -> EXTRA_SURFACE_CLEANUP
            -> SAY_TASK_COMPLETE
        -> succeeded
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)

        # ── Entry: door wait, drive to table ──────────────────────────────────────
        self.add_state(
            "START",
            Start(),
            transitions={
                "succeeded": "SAY_STARTING_CLEANUP",
                "failed":    "failed",
            },
        )

        # Phase 1: table cleanup
        self.add_state(
            "SAY_STARTING_CLEANUP",
            Say(text="I will now clean up the dining table."),
            transitions={
                "succeeded": "TABLE_CLEANUP",
                "aborted":   "TABLE_CLEANUP",
                "canceled":  "TABLE_CLEANUP",
            },
        )

        self.add_state(
            "TABLE_CLEANUP",
            TableCleanup(),
            transitions={
                "succeeded": "SAY_STARTING_BREAKFAST",
                "failed":    "SAY_STARTING_BREAKFAST",  # continue regardless
            },
        )

        # Phase 2: serve breakfast
        self.add_state(
            "SAY_STARTING_BREAKFAST",
            Say(text="I will now set up breakfast."),
            transitions={
                "succeeded": "SERVE_BREAKFAST",
                "aborted":   "SERVE_BREAKFAST",
                "canceled":  "SERVE_BREAKFAST",
            },
        )

        self.add_state(
            "SERVE_BREAKFAST",
            ServeBreakfast(),
            transitions={
                "succeeded": "SAY_STARTING_EXTRA_SURFACE",
                "failed":    "SAY_STARTING_EXTRA_SURFACE",
            },
        )

        # Phase 3: extra surface cleanup
        self.add_state(
            "SAY_STARTING_EXTRA_SURFACE",
            Say(text="I will now check the extra surface."),
            transitions={
                "succeeded": "EXTRA_SURFACE_CLEANUP",
                "aborted":   "EXTRA_SURFACE_CLEANUP",
                "canceled":  "EXTRA_SURFACE_CLEANUP",
            },
        )

        self.add_state(
            "EXTRA_SURFACE_CLEANUP",
            ExtraSurfaceCleanup(),
            transitions={
                "succeeded": "SAY_TASK_COMPLETE",
                "failed":    "SAY_TASK_COMPLETE",
            },
        )

        # Done
        self.add_state(
            "SAY_TASK_COMPLETE",
            Say(
                text="I have completed the pick and place task. "
                     "The table is clean and breakfast is ready."
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
    bb["last_rgb_image"]       = None

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