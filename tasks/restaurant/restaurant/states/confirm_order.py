import yasmin


class ConfirmOrder(yasmin.State):
    """
    STUB: always returns succeeded for testing.
    TODO: say the order back to the customer.
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("order")

    def execute(self, blackboard):
        # STUB: just print the order and return succeeded
        print(f"[STUB] CONFIRM_ORDER: order is {blackboard['order']}")
        return "succeeded"