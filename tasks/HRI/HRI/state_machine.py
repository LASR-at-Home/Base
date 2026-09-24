from threading import Thread

import rclpy
from rclpy.node import Node

import yasmin
import yasmin_ros

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

from lasr_skills import (
    Say,
    SafeGoToLocation,
    StartDoorSM,
    Rotate,
    FollowPerson,
    ReceiveObject,
)

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

        def create_msg(blackboard):
            return String(data="ready")

        self.add_state(
            "START_TABLET",
            yasmin_ros.PublisherState(
                msg_type=String,
                topic_name="/tablet/screen",
                create_message_handler=create_msg,
            ),
            transitions={"succeeded": "WAIT_START"},
        )

        self.add_state(
            "WAIT_START",  # Awaits start Signal for the task
            yasmin_ros.MonitorState(
                topic_name="/tablet/ready",
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
            transitions={"succeeded": "SAY_START", "failed": "START_TIMER"},
        )

        self.add_state(
            "SAY_START",  # SM1: Waits for Door to open, then goes to start
            Say(text="Start of H R I task."),
            transitions={
                "succeeded": "GO_TO_DOOR",
                "canceled": "failed",
                "aborted": "failed",
            },
        )

        self.add_state(
            "GO_TO_DOOR",
            SafeGoToLocation(location_param="door_pose"),
            transitions={"succeeded": "GREET", "failed": "failed"},
        )

        self.add_state(
            "GREET",  # SM2: Greets guest
            LookAndGreetGuest(guest_id="guest1"),
            transitions={"succeeded": "GUIDE_TO_SEAT", "failed": "failed"},
        )

        self.add_state(
            "GUIDE_TO_SEAT",  # GUIDES GUEST TO SEATING AREA
            SafeGoToLocation(location_param="seat_pose"),
            transitions={"succeeded": "SEAT_GUEST", "failed": "failed"},
        )

        self.add_state(
            "SEAT_GUEST",  # SM3: Locates and seats guest in free seat
            SeatGuest(id="guest1"),
            transitions={"succeeded": "CHECK", "failed": "failed"},
        )

        self.add_state(
            "CHECK",
            yasmin.CbState(outcomes=["succeeded", "continue"], callback=self.check),
            transitions={"succeeded": "INTRODUCE", "continue": "GO_TO_DOOR_2"},
        )

        self.add_state(
            "GO_TO_DOOR_2",
            SafeGoToLocation(location_param="door_pose"),
            transitions={"succeeded": "GREET_2", "failed": "failed"},
        )

        self.add_state(
            "GREET_2",  # SM2: Greets guest
            LookAndGreetGuest(guest_id="guest2"),
            transitions={"succeeded": "GUIDE_TO_SEAT_2", "failed": "failed"},
        )

        self.add_state(
            "GUIDE_TO_SEAT_2",  # GUIDES GUEST TO SEATING AREA
            SafeGoToLocation(location_param="seat_pose"),
            transitions={"succeeded": "SEAT_GUEST_2", "failed": "failed"},
        )

        self.add_state(
            "SEAT_GUEST_2",  # SM3: Locates and seats guest in free seat
            SeatGuest(id="guest2"),
            transitions={"succeeded": "CHECK", "failed": "failed"},
        )

        self.add_state(
            "INTRODUCE",
            Introduce(),
            transitions={"succeeded": "GRAB_BAG", "failed": "GRAB_BAG"},
        )

        self.add_state(
            "GRAB_BAG",
            ReceiveObject(object_name="bag"),
            transitions={"succeeded": "ROTATE", "failed": "failed"},
        )

        self.add_state(
            "ROTATE",
            Rotate(angle=180),
            transitions={"succeeded": "ASK_FOR_HOST", "failed": "failed"},
        )

        self.add_state(
            "ASK_FOR_HOST",
            Say(text="Can the host please stand in front of me to lead the way."),
            transitions={
                "succeeded": "FOLLOW_HOST",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "FOLLOW_HOST",
            FollowPerson(),
            transitions={
                "succeeded": "PLACE_BAG",
                "failed": "failed",
            },
        )

        self.add_state(
            "PLACE_BAG",
            PlaceBag(),
            transitions={
                "succeeded": "STOP_TIMER",
                "failed": "failed",
            },
        )

        self.add_state(
            "STOP_TIMER",
            StopTimer(),
            transitions={"succeeded": "SAY_STOP", "failed": "failed"},
        )

        self.add_state(
            "SAY_STOP",
            Say(),
            transitions={
                "succeeded": "SAY_END",
                "aborted": "failed",
                "canceled": "failed",
            },
            remappings={"text": "time_text"},
        )

        self.add_state(
            "SAY_END",
            Say(text="End of h r i task."),
            transitions={
                "succeeded": "succeeded",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

    def check(self, blackboard):
        guest1 = blackboard["guest_data"]["guest1"]
        yasmin.YASMIN_LOG_INFO(f"{self.guest_id}")

        if self.guest_id == 2:
            guest2 = blackboard["guest_data"]["guest2"]
            yasmin.YASMIN_LOG_INFO("Guest1: ")
            for key in guest1.keys():
                value = guest1[key]
                yasmin.YASMIN_LOG_INFO(f"{key}: {value}")
            yasmin.YASMIN_LOG_INFO("Guest2: ")
            for key in guest2.keys():
                value = guest2[key]
                yasmin.YASMIN_LOG_INFO(f"{key}: {value}")
        else:
            yasmin.YASMIN_LOG_INFO("Guest1: ")
            for key in guest1.keys():
                value = guest1[key]
                yasmin.YASMIN_LOG_INFO(f"{key}: {value}")

        self.guest_id += 1
        return "continue" if self.guest_id == 2 else "succeeded"


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
        "host": {"seated_point": None, "seating_detection": False},
        "guest1": {
            "name": "",
            "drink": "",
            "detection": False,
            "seating_detection": False,
            "attributes": {},
            "seated_point": None,
        },
        "guest2": {
            "name": "",
            "drink": "",
            "detection": False,
            "seating_detection": False,
            "attributes": {},
            "seated_point": None,
        },
    }

    drink_detections = {}

    bb["drink_detections"] = drink_detections
    bb["confidence"] = face_detection_confidence
    bb["dataset"] = "hri"
    bb["drink_position"] = PointStamped()
    bb["person_index"] = 0

    bb["z_min"] = -10
    bb["z_sweep_min"] = -10
    bb["z_sweep_max"] = 50
    bb["z_max"] = 50

    outcome = sm(bb)

    yasmin.YASMIN_LOG_INFO(f"State machine has ended with outcome {outcome}")

    # except Exception as e:
    #     yasmin.YASMIN_LOG_WARN(e)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
