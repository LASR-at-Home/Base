from typing import Optional
import yasmin
from yasmin import Blackboard


class GetGuestData(yasmin.State):

    _guest_to_introduce: Optional[str]
    _guest_to_introduce_to: Optional[str]

    def __init__(
        self,
        guest_to_introduce: Optional[str] = None,
        guest_to_introduce_to: Optional[str] = None,
    ):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("guest_data")
        self.add_input_key("named_guest_detection")
        self.add_output_key("relevant_guest_data")
        self.add_output_key("introduce_to")

        """
        Blackboard keys:
         - guest_data: Dictionary of all guests keyed by id (host, guest1, guest2)
           each containing name, drink, and interest.
         - named_guest_detection: Detection result containing the recognised guest id.

        Output keys:
         - relevant_guest_data: Data dictionary for the guest being introduced.
         - introduce_to: Display name (str) of the guest being introduced to.
        """

        self._guest_to_introduce = guest_to_introduce
        self._guest_to_introduce_to = guest_to_introduce_to

    def execute(self, blackboard: Blackboard) -> str:
    guest_data = blackboard["guest_data"]

    # Who we are speaking about
    if self._guest_to_introduce is not None:
        blackboard["relevant_guest_data"] = guest_data[self._guest_to_introduce]
    else:
        reid = blackboard["named_guest_detection"].name
        blackboard["relevant_guest_data"] = guest_data.get(reid, guest_data["host"])

    # Who we are speaking to
    if self._guest_to_introduce_to is not None:
        blackboard["introduce_to"] = guest_data[self._guest_to_introduce_to]["name"]
    else:
        reid = blackboard["named_guest_detection"].name
        if reid not in guest_data:
            blackboard["introduce_to"] = guest_data["host"]["name"]
        else:
            blackboard["introduce_to"] = guest_data[reid]["name"]

    return "succeeded"