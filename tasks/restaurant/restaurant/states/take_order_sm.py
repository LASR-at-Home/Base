import rclpy
import yasmin
from .ask_for_order import AskForOrder
from .confirm_order import ConfirmOrder
from .add_dish import AddDish
from .confirm_full_order import ConfirmFullOrder


class TakeOrderSM(yasmin.StateMachine):
    """
    Sub state machine that handles the full order-taking flow:
    Ask for order → Confirm order → Add dish → Confirm full order → succeeded

    Inputs (from blackboard):
        none

    Outputs (to blackboard):
        order (list[str]): finalised list of 2 items ordered
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

        # 2. Confirm first item — say it back, listen for yes/no
        #    confirmed → ADD_DISH (get second item)
        #    retry     → CONFIRM_ORDER (unclear, ask again)
        #    re_ask    → ASK_FOR_ORDER (wrong, retake first item)
        #    failed    → failed
        self.add_state(
            "CONFIRM_ORDER",
            ConfirmOrder(),
            transitions={
                "confirmed": "ADD_DISH",
                "retry": "CONFIRM_ORDER",
                "re_ask": "ASK_FOR_ORDER",
                "failed": "failed",
            },
        )

        # 3. Add dish — ask for second item and append to order
        self.add_state(
            "ADD_DISH",
            AddDish(node=node),
            transitions={
                "succeeded": "CONFIRM_FULL_ORDER",
                "failed": "failed",
            },
        )

        # 4. Confirm full order — confirm both items, done
        #    confirmed → succeeded
        #    retry     → CONFIRM_FULL_ORDER (unclear)
        #    re_ask    → ADD_DISH (wrong second item)
        #    failed    → failed
        self.add_state(
            "CONFIRM_FULL_ORDER",
            ConfirmFullOrder(),
            transitions={
                "confirmed": "succeeded",
                "retry": "CONFIRM_FULL_ORDER",
                "re_ask": "ADD_DISH",
                "failed": "failed",
            },
        )


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node(
        node_name="restaurant",
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
