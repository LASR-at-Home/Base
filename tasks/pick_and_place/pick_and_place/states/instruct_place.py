import yasmin
import yasmin_ros

from lasr_skills import Say


class InstructPlace(yasmin.State):
    """
    Instructs the human operator where to place the selected object.

    Reads chosen_shelf and chosen_shelf_str from the blackboard.
    chosen_shelf_str is a placement hint set by ChooseShelf e.g.
    "near the cereal" — if empty the instruction omits the hint.

    Blackboard inputs:
        selected_object_name : str
        chosen_shelf         : str
        chosen_shelf_str     : str
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("selected_object_name")
        self.add_input_key("chosen_shelf")
        self.add_input_key("chosen_shelf_str")

    def execute(self, blackboard) -> str:
        name             = blackboard["selected_object_name"]
        chosen_shelf     = blackboard["chosen_shelf"]
        chosen_shelf_str = blackboard["chosen_shelf_str"]

        yasmin.YASMIN_LOG_INFO(
            f"Instructing place: {name} on {chosen_shelf} {chosen_shelf_str}"
        )

        if chosen_shelf_str:
            text = (
                f"Please place the {name} on {chosen_shelf}, "
                f"{chosen_shelf_str}. "
                f"I will give you 5 seconds. 5.. 4.. 3.. 2.. 1.."
            )
        else:
            text = (
                f"Please place the {name} on {chosen_shelf}. "
                f"I will give you 5 seconds. 5.. 4.. 3.. 2.. 1.."
            )

        say = Say(text=text)
        outcome = say.execute(blackboard)

        if outcome in ("succeeded", "aborted"):
            return "succeeded"

        return "failed"