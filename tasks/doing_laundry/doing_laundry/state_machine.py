from threading import Thread
import rclpy
from rclpy.node import Node
import yasmin
import yasmin_ros
from doing_laundry.states import Start, LookForBasket

try:
    from rclpy.executors import EventsExecutor as Executor
except ImportError:
    from rclpy.executors import MultiThreadedExecutor as Executor


class DoingLaundry(yasmin.StateMachine):
    """
    Main state machine for the Doing Laundry task.

    Flow:
        START
            → WAIT_START           (wait for /doing_laundry/start signal)
            → SAY_START            (announce task start)
            → SAY_WAITING          (announce waiting for door)
            → WAIT_FOR_DOOR        (detect door opening via LIDAR)
            → GO_TO_LAUNDRY_AREA   (navigate to laundry area)
        → LOOK_FOR_BASKET
            → rotate + head sweep + open vocab basket detection
        → succeeded / failed
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)

        self.add_state(
            "START",
            Start(),
            transitions={
                "succeeded": "LOOK_FOR_BASKET",
                "failed": "failed",
            },
        )

        self.add_state(
            "LOOK_FOR_BASKET",
            LookForBasket(),
            transitions={
                "found": "succeeded",
                "not_found": "succeeded",  # proceed anyway for now
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
    bb["basket_detection"] = None  # ← added
    try:
        outcome = sm(bb)
        yasmin.YASMIN_LOG_INFO(f"Doing Laundry finished with outcome: {outcome}")
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(str(e))
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()