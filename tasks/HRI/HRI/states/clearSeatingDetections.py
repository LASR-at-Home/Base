import smach
from smach import UserData

class ClearSeatingDetections(smach.State):
    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data"],
            output_keys=["guest_data"],
        )

    def execute(self, userdata: UserData) -> str:
        """
        Clears the seating detection for all guests in the guest data.
        This is to ensure that we can re-detect guests when they are seated.
        """
        for guest_id in userdata.guest_data:
            userdata.guest_data[guest_id]["seating_detection"] = False
        return "succeeded"
