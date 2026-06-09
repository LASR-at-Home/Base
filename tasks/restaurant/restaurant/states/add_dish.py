import yasmin


class AddDish(yasmin.State):
    """
    STUB: always returns succeeded for testing.
    TODO: listen for second item and append to order list using LLM.
    """

    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("order")
        self.add_output_key("order")
        self.add_output_key("transcribed_speech")

    def execute(self, blackboard):
        # STUB: pretend customer ordered a cola as second item
        print(f"[STUB] ADD_DISH: adding 'cola' to order {blackboard['order']}")
        blackboard["order"] = blackboard["order"] + ["cola"]
        print(f"[STUB] ADD_DISH: order is now {blackboard['order']}")
        return "succeeded"