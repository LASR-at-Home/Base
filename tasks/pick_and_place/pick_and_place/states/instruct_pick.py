import yasmin

from lasr_skills import Say


class InstructPick(yasmin.State):
    """
    Instructs the human operator to pick up the selected object, and announces
    the robot's classification + intended destination (Communicating Perception).

    Blackboard inputs:
        selected_object_name : str
        object_category      : str
        destination_str      : str   — e.g. "the dishwasher"
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("selected_object_name")
        self.add_input_key("object_category")
        self.add_input_key("destination_str")

    def execute(self, blackboard) -> str:
        name            = blackboard["selected_object_name"]
        category        = blackboard["object_category"] or "unknown"
        destination_str = blackboard["destination_str"] or "its place"

        yasmin.YASMIN_LOG_INFO(
            f"Instructing pick: {name} ({category}) -> {destination_str}"
        )

        text = (
            f"I have selected the {name}. "
            f"I classified it as a {category} item, so it goes to {destination_str}. "
            f"Please pick it up and hold it ready."
        )

        say = Say(text=text)
        outcome = say.execute(blackboard)

        if outcome in ("succeeded", "aborted"):
            return "succeeded"

        return "failed"