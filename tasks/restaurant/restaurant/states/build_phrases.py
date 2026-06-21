import yasmin


def build_sentence_from_list(order):
    """
    Builds an 'and' separated list.
    e.g. ['pizza', 'pasta', 'salad'] -> "pizza, pasta and salad"
    """
    if not order:
        return ""
    elif len(order) == 1:
        return order[0]
    else:
        return ", ".join(order[:-1]) + " and " + order[-1]


class BuildPlaceOrderPhrase(yasmin.State):
    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("order")
        self.add_output_key("place_order_phrase")

    def execute(self, blackboard):
        order = blackboard["order"]

        order_str = build_sentence_from_list(order)
        blackboard["place_order_phrase"] = f"I would like to order {order_str}."
        return "succeeded"


class BuildAnnounceOrderPhrase(yasmin.State):
    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("order")
        self.add_output_key("announce_order_phrase")

    def execute(self, blackboard):
        order = blackboard["order"]

        order_str = build_sentence_from_list(order)
        blackboard["announce_order_phrase"] = (
            f"Your order is ready. Please collect {order_str}. Enjoy!"
        )
        return "succeeded"
