import yasmin
from yasmin import Blackboard


class ClearSeatingDetections(yasmin.State):
    """
    Clears the seating detection for all guests in the guest data.
    This is to ensure that we can re-detect guests when they are seated.
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("guest_data")
        self.add_output_key("guest_data")

    def execute(self, blackboard: Blackboard) -> str:
        blackboard["seat_indexes"] = {"guest1": None, "guest2": None, "host": None}
        for guest_id in blackboard["guest_data"]:
            blackboard["guest_data"][guest_id]["seating_detection"] = False
        return "succeeded"
