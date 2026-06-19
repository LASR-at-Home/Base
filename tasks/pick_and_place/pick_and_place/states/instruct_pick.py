import yasmin
import yasmin_ros

from lasr_skills import Say


class InstructPick(yasmin.State):
    """
    Instructs the human operator to pick up the selected object.

    Reads selected_object_name from the blackboard and speaks a pick
    instruction via TTS.

    Blackboard inputs:
        selected_object_name : str
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("selected_object_name")

    def execute(self, blackboard) -> str:
        name = blackboard["selected_object_name"]

        yasmin.YASMIN_LOG_INFO(f"Instructing pick: {name}")

        text = (
            f"I have selected the {name}. "
            f"Please pick it up and hold it ready."
        )

        say = Say(text=text)
        outcome = say.execute(blackboard)

        if outcome in ("succeeded", "aborted"):
            return "succeeded"

        return "failed"