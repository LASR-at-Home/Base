from typing import List, Tuple, Dict

from threading import Thread

import rclpy
from rclpy.node import Node

import yasmin
import yasmin_ros

from geometry_msgs.msg import Point, PointStamped, Pose

from lasr_skills import Say, GoToLocation, StopEyeTracker

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

        def wait_cb(blackboard, msg):
            yasmin.YASMIN_LOG_INFO("RECEIVED START SIGNAL")
            return "succeeded"

        self.add_state(
            "WAIT_START",  # Awaits start Signal for the task
            yasmin_ros.MonitorState(
                topic_name="/receptionist/start",
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
            "SAY_START",
            Say(text='Start of h r i task'),
            transitions={'succeeded': 'GO_TO_START', 'aborted': 'failed', 'canceled': 'failed'}
        )

        # self.add_state(
        #     "START_CON",  # SM1: Waits for Door to open, then goes to start
        #     self.setup(),
        #     transitions={"succeeded": "GREET", "failed": "START_CON"},
        # )

        self.add_state(
            "GO_TO_START",  
            GoToLocation(location_param="start_pose"),
            transitions={"succeeded": "GO_TO_DOOR", "failed": "failed"},
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

        self.add_state('STOP_EYE_TRACKER',
                       StopEyeTracker(),
                       transitions={'succeeded': 'SAY_FOLLOW', 'aborted': 'SAY_FOLLOW', 'canceled': 'SAY_FOLLOW', 'timeout': 'SAY_FOLLOW'})

        self.add_state('SAY_FOLLOW',
                       Say(text='Welcome. Follow me to the seating area.'),
                       transitions={
                            "succeeded": "GUIDE_TO_SEAT",
                            "aborted": "GUIDE_TO_SEAT",
                            "canceled": "GUIDE_TO_SEAT",
                        })

        self.add_state(
            "GUIDE_TO_SEAT",  # GUIDES GUEST TO SEATING AREA
            GoToLocation(location_param="seat_pose"),
            transitions={"succeeded": "SEAT_GUEST", "failed": "failed"},
        )

        self.add_state(
            "SEAT_GUEST",  # SM3: Locates and seats guest in free seat
            SeatGuest(learn_host=False),
            transitions={"succeeded": "succeeded", "failed": "failed"},
        )

    def setup(self):
        start_con_sm = yasmin.Concurrence(
            states={
                "SAY_START": Say(text="Start of HRI task."),
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
        },
        "guest2": {
            "name": "",
            "drink": "",
            "detection": False,
            "seating_detection": False,
        },
    }

    drink_detections = {}

    bb["drink_detections"] = drink_detections
    bb["confidence"] = face_detection_confidence
    bb["dataset"] = "hri"
    bb["drink_position"] = PointStamped()

    outcome = sm(bb)

    yasmin.YASMIN_LOG_INFO(f"State machine has ended with outcome {outcome}")

    # except Exception as e:
    #     yasmin.YASMIN_LOG_WARN(e)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
