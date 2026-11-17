import time

import yasmin

from lasr_skills import Say


class InstructPick(yasmin.State):
    """
    Asks the person to take the selected object and follow the robot to its destination
    (the robot does not grasp in this demo).

    Blackboard inputs:
        selected_object_name : str
        object_category      : str
        destination_str      : str   - e.g. "the dishwasher"
    """

    WAIT_S = 3.0   # time to pick the object up before the robot drives off

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
        text = f"Please grab the {name} and follow me to {destination_str}."

        outcome = Say(text=text).execute(blackboard)
        time.sleep(self.WAIT_S)
        if outcome in ("succeeded", "aborted"):
            return "succeeded"
        return "failed"
