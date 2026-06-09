import yasmin


class AnythingElse(yasmin.State):
    """
    STUB: always returns nothing for testing.
    TODO: ask customer if they want anything else.
    """

    def __init__(self):
        super().__init__(outcomes=["nothing", "add_dish", "failed"])
        self.add_output_key("transcribed_speech")

    def execute(self, blackboard):
        # STUB: pretend customer wants nothing else
        print("[STUB] ANYTHING_ELSE: customer wants nothing else")
        return "nothing"