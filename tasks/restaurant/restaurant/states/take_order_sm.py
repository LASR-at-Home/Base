import rclpy
import yasmin

from .ask_for_order import AskForOrder
from .confirm_order import ConfirmOrder
from .repeat_order import RepeatOrder
from .add_dish import AddDish
from .anything_else import AnythingElse


class TakeOrderSM(yasmin.StateMachine):
    """
    Sub state machine that handles the full order-taking flow:
    Ask for order → Confirm order → Repeat? → Anything else? → Add dish (loop)

    Inputs (from blackboard):
        none

    Outputs (to blackboard):
        order (list[str]): finalised list of items ordered
    """

    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "failed"])

        # Keys written to the blackboard
        self.add_output_key("order")

        # 1. Ask for order — ask customer what they want and listen
        self.add_state(
            "ASK_FOR_ORDER",
            AskForOrder(node=node),
            transitions={
                "succeeded": "CONFIRM_ORDER",
                "failed": "failed",
            },
        )

        # 2. Confirm order — robot reads back what it heard
        self.add_state(
            "CONFIRM_ORDER",
            ConfirmOrder(),
            transitions={
                "succeeded": "REPEAT_ORDER",
                "failed": "failed",
            },
        )

        # 3. Repeat order? — did the customer say yes/correct or no/repeat?
        self.add_state(
            "REPEAT_ORDER",
            RepeatOrder(),
            transitions={
                "correct": "ANYTHING_ELSE",
                "repeat": "CONFIRM_ORDER",
                "failed": "failed",
            },
        )

        # 4. Anything else? — does the customer want to add anything?
        self.add_state(
            "ANYTHING_ELSE",
            AnythingElse(),
            transitions={
                "nothing": "succeeded",
                "add_dish": "ADD_DISH",
                "failed": "failed",
            },
        )

        # 5. Add dish — listen for the second item and append to order
        self.add_state(
            "ADD_DISH",
            AddDish(node=node),
            transitions={
                "succeeded": "CONFIRM_ORDER",
                "failed": "failed",
            },
        )


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node(
        node_name="take_order_sm",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )
    sm = TakeOrderSM(node=node)
    outcome = sm(yasmin.Blackboard())
    node.get_logger().info(f"TakeOrderSM outcome: {outcome}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
