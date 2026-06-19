import yasmin

from lasr_skills import Say


class InstructPlace(yasmin.State):
    """
    Instructs the human operator where to place the selected object.

    The destination phrase comes from DecideDestination (the dishwasher / the
    trash bin / the cabinet). For cabinet placements, ChooseShelf additionally
    provides a shelf id (chosen_shelf) and an optional hint (chosen_shelf_str
    e.g. "near the cereal"). Dishwasher / trash placements carry no shelf hint.

    Blackboard inputs:
        selected_object_name : str
        destination_str      : str
        chosen_shelf         : str
        chosen_shelf_str     : str
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("selected_object_name")
        self.add_input_key("destination_str")
        self.add_input_key("chosen_shelf")
        self.add_input_key("chosen_shelf_str")

    def execute(self, blackboard) -> str:
        name             = blackboard["selected_object_name"]
        destination_str  = blackboard["destination_str"] or "its place"
        chosen_shelf     = blackboard["chosen_shelf"]
        chosen_shelf_str = blackboard["chosen_shelf_str"]

        # Build an optional shelf hint (cabinet only).
        hint_parts = []
        if chosen_shelf:
            hint_parts.append(f"on {chosen_shelf}")
        if chosen_shelf_str:
            hint_parts.append(chosen_shelf_str)
        hint = (", " + ", ".join(hint_parts)) if hint_parts else ""

        yasmin.YASMIN_LOG_INFO(
            f"Instructing place: {name} in {destination_str}{hint}"
        )

        text = (
            f"Please place the {name} in {destination_str}{hint}. "
        )

        say = Say(text=text)
        outcome = say.execute(blackboard)

        if outcome in ("succeeded", "aborted"):
            return "succeeded"

        return "failed"