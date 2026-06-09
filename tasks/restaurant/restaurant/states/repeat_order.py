import yasmin


class RepeatOrder(yasmin.State):
    """
    STUB: always returns correct for testing.
    TODO: listen for customer confirmation.
    """

    def __init__(self):
        super().__init__(outcomes=["correct", "repeat", "failed"])

    def execute(self, blackboard):
        # STUB: pretend customer said yes
        print("[STUB] REPEAT_ORDER: customer said correct")
        return "correct"