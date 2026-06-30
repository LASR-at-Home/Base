from time import sleep, time
from typing import List

import yasmin
import yasmin_ros
from lasr_skills import Say
from robot_ui.msg import Order
from std_msgs.msg import String

# Maps competition object IDs to human-readable names for TTS phrases.
# Matches the ITEMS list in the robot_ui CreateOrder component.
ITEM_NAMES = {
    "juice_pack": "juice pack",
    "cola": "cola",
    "milk": "milk",
    "orange_juice": "orange juice",
    "tropical_juice": "tropical juice",
    "red_wine": "red wine",
    "iced_tea": "iced tea",
    "orange": "orange",
    "pear": "pear",
    "peach": "peach",
    "strawberry": "strawberry",
    "apple": "apple",
    "lemon": "lemon",
    "banana": "banana",
    "plum": "plum",
    "cornflakes": "cornflakes",
    "pringles": "pringles",
    "cheezit": "cheez-it",
    "chocolate_jello": "chocolate jello",
    "coffee_grounds": "coffee grounds",
    "mustard": "mustard",
    "tomato_soup": "tomato soup",
    "tuna": "tuna",
    "strawberry_jello": "strawberry jello",
    "spam": "spam",
    "sugar": "sugar",
}


def ids_to_names(ids: List[str]) -> List[str]:
    return [ITEM_NAMES.get(item_id, item_id.replace("_", " ")) for item_id in ids]


class _WaitForTabletOrder(yasmin.State):
    """
    Publishes "order" to /tablet/screen, then blocks until the customer
    submits their selection via /tablet/order.
    """

    def __init__(self, timeout: float):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_output_key("order")
        self._timeout = timeout
        self._order_data = None

        node = yasmin_ros.logger_node
        self._screen_pub = node.create_publisher(String, "/tablet/screen", 10)
        node.create_subscription(Order, "/tablet/order", self._order_cb, 10)

    def _order_cb(self, msg: Order) -> None:
        if self._order_data is None:
            self._order_data = list(msg.products)

    def execute(self, blackboard):
        self._order_data = None

        screen_msg = String()
        screen_msg.data = "order"
        self._screen_pub.publish(screen_msg)

        deadline = time() + self._timeout
        while self._order_data is None:
            if time() > deadline:
                yasmin.YASMIN_LOG_WARN("Timed out waiting for tablet order")
                return "failed"
            sleep(0.5)

        names = ids_to_names(self._order_data)
        blackboard["order"] = names
        yasmin.YASMIN_LOG_INFO(f"Tablet order received: {self._order_data} -> {names}")

        home_msg = String()
        home_msg.data = "home"
        self._screen_pub.publish(home_msg)

        return "succeeded"


class TakeOrderTablet(yasmin.StateMachine):
    """
    Takes the customer's order via the robot_ui tablet interface.

    The robot asks the customer to use the tablet, then waits for them to
    submit their selection. The tablet UI includes its own confirmation step
    so no additional verbal confirm is needed.

    Outputs (to blackboard):
        order (list[str]): human-readable item names, e.g. ["cola", "apple", "orange juice"]

    Outcomes:
        succeeded — order received from tablet
        failed    — timed out waiting for input
    """

    def __init__(self, timeout: float = 120.0):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_output_key("order")

        self.add_state(
            "SAY_USE_TABLET",
            Say(text="Please use the tablet to place your order."),
            transitions={
                "succeeded": "WAIT_FOR_ORDER",
                "aborted": "WAIT_FOR_ORDER",
                "canceled": "WAIT_FOR_ORDER",
            },
        )

        self.add_state(
            "WAIT_FOR_ORDER",
            _WaitForTabletOrder(timeout=timeout),
            transitions={
                "succeeded": "succeeded",
                "failed": "failed",
            },
        )
