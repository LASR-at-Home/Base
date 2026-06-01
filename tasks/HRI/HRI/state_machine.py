from typing import List, Tuple, Dict

import rclpy
import yasmin
import yasmin_ros

from geometry_msgs.msg import Point, PointStamped, Pose

from lasr_skills import Say, GoToLocation

from HRI.states import *

from shapely.geometry import Polygon
from std_msgs.msg import Empty


class HRI(yasmin.StateMachine):
    def __init__(self, host_data, face_detection_confidence=0.2):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)

        def wait_cb(blackboard, msg):
            yasmin.YASMIN_LOG_INFO('RECEIVED START SIGNAL')
            return 'succeeded'

        with self:
            self.blackboard["guest_data"] = {
                "host": host_data,
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

            self.blackboard["drink_detections"] = drink_detections
            self.blackboard["confidence"] = face_detection_confidence
            self.blackboard["dataset"] = "receptionist"
            self.blackboard["drink_position"] = PointStamped()

            self.add_state(
                "WAIT_START",  # Awaits start Signal for the task
                yasmin_ros.MonitorState(
                    topic="/receptionist/start",
                    outcomes=['succeeded', 'failed'],
                    monitor_handler=wait_cb,
                    msg_type=Empty,
                ),
                transitions={
                    "succeeded": "START_TIMER",
                    "failed": "WAIT_START",
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
                transitions={"succeeded": "GREET", "failed": "START_CON"},
            )

            self.add(
                "GREET",  # SM2: Greets guest
                LookAndGreetGuest(node=node, last_resort=False, guest_id="guest1"),
                transitions={"succeeded": "GUIDE_TO_SEAT", "failed": "failed"},
            )

            self.add(
                "GUIDE_TO_SEAT",  # GUIDES GUEST TO SEATING AREA
                GoToLocation(node=node, location_param="seat_pose"),
                transitions={"succeeded": "SEAT_GUEST", "failed": "failed"},
            )

            self.add(
                "SEAT_GUEST",  # SM3: Locates and seats guest in free seat
                SeatGuest(node=node, learn_host=False),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )

    def setup(self, node):
        start_con_sm = yasmin.Concurrence(
            states={
                "SAY_START": Say(node=node, text="Start of HRI task."),
                "DOOR_START": StartDoorSM(node=node),
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


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node(
        node_name="hri",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )

    sm = HRI(node=node, host_data={})
    outcome = sm.execute()
    node.get_logger().info(f"StartSM outcome: {outcome}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
