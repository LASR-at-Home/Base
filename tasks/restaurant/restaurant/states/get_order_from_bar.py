"""
Going to bar
Face barman
Say order to Barman
Wait for order to be placed on tray (time can be constant, e.g. 30 sec)
Navigate back to table
Face guests
Announce order
"""

import yasmin
import yasmin_ros
from geometry_msgs.msg import Pose, Point, Quaternion
from lasr_skills import SafeGoToLocation, Say, Wait, PlayMotion
from restaurant.states import FaceCustomer
from restaurant.states.build_phrases import (
    BuildPlaceOrderPhrase,
    BuildAnnounceOrderPhrase,
)


class GetOrderFromBar(yasmin.StateMachine):

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("order")
        self.add_input_key("bar_pose")

        self.node = yasmin_ros.logger_node

        parameters = {
            "wait_duration": self.get_value("wait_duration"),
            "ordered_food": self.get_value("ordered_food"),
        }

        yasmin.YASMIN_LOG_INFO(f"parameters {parameters}")

        self.add_state(
            "GO_TO_BAR",
            SafeGoToLocation(),
            transitions={"succeeded": "BUILD_PLACE_ORDER", "failed": "failed"},
            remappings={"location_param": "bar_pose"},
        )

        self.add_state(
            "BUILD_PLACE_ORDER",
            BuildPlaceOrderPhrase(),
            transitions={"succeeded": "PLACE_ORDER", "failed": "failed"},
        )

        self.add_state(
            "PLACE_ORDER",
            Say(),
            transitions={
                "succeeded": "SAY_ROTATE_GET_ORDER",
                "aborted": "failed",
                "canceled": "failed",
            },
            remappings={"text": "place_order_phrase"},
        )

        self.add_state(
            "SAY_ROTATE_GET_ORDER",
            Say(text="I will now rotate so you put the items in the basket and then wait for 10 seconds."),
            transitions={
                "succeeded": "ROTATE_GET_ORDER",
                "aborted": "failed",
                "canceled": "failed",
            }
        )

        self.add_state(
            "ROTATE_GET_ORDER",
            Rotate(angle=180),
            transitions={"succeeded": "WAIT_FOR_ORDER", "failed": "failed"},
        )

        self.add_state(
            "WAIT_FOR_ORDER",
            Wait(wait_time=parameters["wait_duration"]),
            transitions={"succeeded": "GO_TO_TABLE", "failed": "failed"},
        )

        self.add_state(
            "GO_TO_TABLE",
            SafeGoToLocation(),
            transitions={"succeeded": "FACE_CUSTOMER", "failed": "failed"},
            remappings={"location_param": "location"},
        )

        self.add_state(
            "FACE_CUSTOMER",
            FaceCustomer(),
            transitions={"succeeded": "LOOK_AT_CUSTOMER", "failed": "LOOK_AT_CUSTOMER"},
        )

        self.add_state(
            "LOOK_AT_CUSTOMER",
            PlayMotion(motion_name="look_centre"),
            transitions={
                "succeeded": "BUILD_ANNOUNCE_ORDER_PHRASE",
                "aborted": "BUILD_ANNOUNCE_ORDER_PHRASE",
                "canceled": "BUILD_ANNOUNCE_ORDER_PHRASE",
            },
        )

        self.add_state(
            "BUILD_ANNOUNCE_ORDER_PHRASE",
            BuildAnnounceOrderPhrase(),
            transitions={"succeeded": "ANNOUNCE_ORDER", "failed": "failed"},
        )

        self.add_state(
            "ANNOUNCE_ORDER",
            Say(),
            transitions={
                "succeeded": "SAY_ROTATE_COLLECT_ORDER",
                "aborted": "failed",
                "canceled": "failed",
            },
            remappings={"text": "announce_order_phrase"},
        )

        self.add_state(
            "SAY_ROTATE_COLLECT_ORDER",
            Say(text="I will now rotate so you can collect it within 10 seconds"),
            transitions={
                "succeeded": "ROTATE_COLLECT_ORDER",
                "aborted": "failed",
                "canceled": "failed",
            }
        )

        self.add_state(
            "ROTATE_COLLECT_ORDER",
            Rotate(angle=180),
            transitions={"succeeded": "WAIT_FOR_COLLECTION", "failed": "failed"},
        )

        self.add_state(
            "WAIT_FOR_COLLECTION",
            Wait(wait_time=parameters["wait_duration"]),
            transitions={"succeeded": "succeeded", "failed": "failed"},
        )

    def get_value(self, key):
        return self.node.get_parameter(key).value
