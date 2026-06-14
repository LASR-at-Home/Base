from typing import List, Tuple, Dict

from threading import Thread

import rclpy
from rclpy.node import Node

import yasmin
import yasmin_ros

from geometry_msgs.msg import Point, PointStamped, Pose

from lasr_skills import Say, GoToLocation, StopEyeTracker, PlayMotion

from HRI.states import *

from shapely.geometry import Polygon
from std_msgs.msg import Empty

try:
    from rclpy.executors import EventsExecutor as Executor
except ImportError:
    from rclpy.executors import MultiThreadedExecutor as Executor


class HRI(yasmin.StateMachine):
    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)

        self.guest_id = 1

        def wait_cb(blackboard, msg):
            yasmin.YASMIN_LOG_INFO("RECEIVED START SIGNAL")
            return "succeeded"

        self.add_state(
            "WAIT_START",  # Awaits start Signal for the task
            yasmin_ros.MonitorState(
                topic_name="/hri/start",
                outcomes=["succeeded", "failed"],
                monitor_handler=wait_cb,
                msg_type=Empty,
            ),
            transitions={
                "succeeded": "START_TIMER",
                "failed": "WAIT_START",
                "canceled": "failed",
            },
        )

        self.add_state(
            "START_TIMER",
            StartTimer(),
            transitions={"succeeded": "START_CON", "failed": "START_TIMER"},
        )

        self.add_state(
            "START_CON",  # SM1: Waits for Door to open, then goes to start
            self.setup(),
            transitions={"succeeded": "GO_TO_DOOR", "failed": "START_CON"},
        )

        self.add_state(
            "GO_TO_DOOR",
            GoToLocation(location_param="door_pose"),
            transitions={"succeeded": "GREET", "failed": "failed"},
        )

        self.add_state(
            "GREET",  # SM2: Greets guest
            LookAndGreetGuest(last_resort=False, guest_id="guest1"),
            transitions={"succeeded": "STOP_EYE_TRACKER", "failed": "failed"},
        )

        self.add_state(
            "STOP_EYE_TRACKER",
            StopEyeTracker(),
            transitions={
                "succeeded": "LOOK_CENTRE",
                "aborted": "failed",
                "canceled": "failed",
                "timeout": "failed",
            },
        )

        self.add_state(
            "LOOK_CENTRE",
            PlayMotion("look_centre"),
            transitions={
                "succeeded": "SAY_FOLLOW",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "SAY_FOLLOW",
            Say(text="Welcome. Follow me to the seating area."),
            transitions={
                "succeeded": "GUIDE_TO_SEAT",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "GUIDE_TO_SEAT",  # GUIDES GUEST TO SEATING AREA
            GoToLocation(location_param="seat_pose"),
            transitions={"succeeded": "SEAT_GUEST", "failed": "failed"},
        )

        self.add_state(
            "SEAT_GUEST",  # SM3: Locates and seats guest in free seat
            SeatGuest(learn_host=False),
            transitions={"succeeded": "CHECK", "failed": "failed"},
        )

        self.add_state(
            "CHECK",
            yasmin.CbState(outcomes=["succeeded", 'GO_TO_DOOR_2'], callback=self.check),
            transitions={"succeeded": 'INTRODUCE', 'GO_TO_DOOR_2': 'GO_TO_DOOR_2'},
        )

        self.add_state(
            "GO_TO_DOOR_2",
            GoToLocation(location_param="door_pose"),
            transitions={"succeeded": "GREET_2", "failed": "failed"},
        )

        self.add_state(
            "GREET_2",  # SM2: Greets guest
            LookAndGreetGuest(last_resort=False, guest_id="guest2"),
            transitions={"succeeded": "STOP_EYE_TRACKER", "failed": "failed"},
        )
        
        self.add_state(
            "INTRODUCE",
            Introduce(guest_to_introduce="guest1"),
            transitions={"succeeded": "succeeded", "failed": "failed"},
        )

    def check(self, blackboard):
        guest = blackboard["guest_data"][f"guest{self.guest_id}"]
        yasmin.YASMIN_LOG_INFO(f"{self.guest_id}")

        for key in guest.keys():
            value = guest[key]
            yasmin.YASMIN_LOG_INFO(f"{key}: {value}")

        self.guest_id += 1
        return "GO_TO_DOOR_2" if self.guest_id == 2 else "succeeded"

    def setup(self):
        start_con_sm = yasmin.Concurrence(
            states={
                "SAY_START": Say(text="Start of H R I task."),
                "DOOR_START": StartDoorSM(),
            },
            default_outcome="failed",
            outcome_map={
                "succeeded": {
                    "SAY_START": "succeeded",
                    "DOOR_START": "succeeded",
                },
                "failed": {
                    "SAY_START": "aborted",
                    "DOOR_START": "failed",
                },
            },
        )

        return start_con_sm


class HRI_node(Node):
    def __init__(self):
        super().__init__(
            node_name="hri",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self._executor = Executor()
        self._executor.add_node(self)
        self._spin_thread = Thread(target=self._executor.spin)
        self._spin_thread.start()


def main():
    rclpy.init()

    node = HRI_node()

    yasmin_ros.set_ros_loggers(node)

    sm = HRI()
    bb = yasmin.Blackboard()

    face_detection_confidence = 0.2

    bb["guest_data"] = {
        "guest1": {
            "name": "",
            "drink": "",
            "detection": False,
            "seating_detection": False,
            "attributes": {},
        },
        "guest2": {
            "name": "",
            "drink": "",
            "detection": False,
            "seating_detection": False,
            "attributes": {},
        },
    }

    drink_detections = {}

    bb["drink_detections"] = drink_detections
    bb["confidence"] = face_detection_confidence
    bb["dataset"] = "hri"
    bb["drink_position"] = PointStamped()
    bb["person_index"] = 0

    outcome = sm(bb)

    yasmin.YASMIN_LOG_INFO(f"State machine has ended with outcome {outcome}")

    # except Exception as e:
    #     yasmin.YASMIN_LOG_WARN(e)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
