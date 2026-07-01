import os
from threading import Thread

import rclpy
from rclpy.node import Node

import yasmin
import yasmin_ros
try:
    from yasmin_viewer import YasminViewerPub
except Exception:
    YasminViewerPub = None
from ament_index_python.packages import get_package_share_directory

from doing_laundry.states import (
    TuckArm,
    Spawn,
    LookDown,
    DetectBasket,
    Pick,
    Move,
    Place,
)

try:
    from rclpy.executors import EventsExecutor as Executor
except ImportError:
    from rclpy.executors import MultiThreadedExecutor as Executor


class DoingLaundry(yasmin.StateMachine):
    """
    Laundry pick-and-place.

    Default flow (perception test, no MoveIt needed):
        SPAWN -> DETECT_BASKET -> succeeded

    Full flow (uncomment Pick/Move/Place below + their imports):
        SPAWN -> DETECT_BASKET -> PICK -> MOVE_TO_DESK -> PLACE -> succeeded
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)

        basket_sdf = os.path.join(
            get_package_share_directory("doing_laundry"), "models", "basket.sdf")

        self.add_state(
            "SPAWN",
            Spawn(model_path=basket_sdf, x=0.6, y=0.0, z=0.0, settle=1.5),
            transitions={"succeeded": "TUCK_ARM", "failed": "TUCK_ARM"})

        self.add_state(
            "TUCK_ARM",
            TuckArm(),
            transitions={"succeeded": "LOOK_DOWN", "failed": "LOOK_DOWN"})

        self.add_state(
            "LOOK_DOWN",
            LookDown(tilt=-0.9),
            transitions={"succeeded": "DETECT_BASKET", "failed": "DETECT_BASKET"})
        
        self.add_state(
            "DETECT_BASKET",
            DetectBasket(gripper_half=0.07, safe_margin=0.02),
            transitions={
                "grasp_ready": "PICK",
                "empty": "succeeded",
                "no_basket": "failed",
                "failed": "failed",
            })

        # ── Full manipulation flow (needs MoveIt + pymoveit2 running) ────────
        self.add_state(
            "PICK", Pick(),
            transitions={"succeeded": "MOVE_TO_DESK", "failed": "DETECT_BASKET"})
        self.add_state(
            "MOVE_TO_DESK", Move(x=0.6, y=-0.3, z=0.95),
            transitions={"succeeded": "PLACE", "failed": "failed"})
        self.add_state(
            "PLACE", Place(x=0.6, y=-0.3, z=0.95),
            transitions={"succeeded": "succeeded", "failed": "failed"})


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

    # Uncomment to visualise the state machine in the browser viewer
    # YasminViewerPub(sm)

    bb = yasmin.Blackboard()

    try:
        outcome = sm(bb)
        yasmin.YASMIN_LOG_INFO(f"Doing laundry finished with outcome: {outcome}")
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(str(e))

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
