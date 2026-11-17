import time

import yasmin

from lasr_skills import Say


class InstructPlace(yasmin.State):
    """
    At the destination: asks the person to put the object there
    ("on the shelf" for the cabinet, "in the dishwasher", "in the trash bin").

    Blackboard inputs:
        selected_object_name : str
        destination_str      : str
        chosen_shelf         : str   (not spoken: no shelf perception in this demo)
        chosen_shelf_str     : str
    """

    WAIT_S = 5.0   # time to put the object down before the robot drives back

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("selected_object_name")
        self.add_input_key("destination_str")
        self.add_input_key("chosen_shelf")
        self.add_input_key("chosen_shelf_str")

    def execute(self, blackboard) -> str:
        name            = blackboard["selected_object_name"]
        destination_str = blackboard["destination_str"] or "its place"
        where = "on the shelf" if destination_str == "the cabinet" else f"in {destination_str}"

        yasmin.YASMIN_LOG_INFO(f"Instructing place: {name} {where}")
        text = f"Please put the {name} {where}."

        outcome = Say(text=text).execute(blackboard)
        time.sleep(self.WAIT_S)
        if outcome in ("succeeded", "aborted"):
            return "succeeded"
        return "failed"
