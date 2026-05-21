from typing import Optional

import smach
from smach import UserData

class GetGuestData(smach.State):
    _guest_to_introduce: Optional[str]
    _guest_to_introduce_to: Optional[str]

    def __init__(
            self,
            guest_to_introduce: Optional[str] = None,
            guest_to_introduce_to: Optional[str] = None,
    ):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data", "named_guest_detection"],
            output_keys=["relevant_guest_data", "introduce_to"],
        )

        """
        Input keys are :
         - guest_data: A dictionary containing the data of all guests, where the keys are
           the ids (host, guest1, guest2) of the guests and the values are dictionaries with 
           their data (name, drink, interest).
         - named_guest_detection: The detection of the guest to introduce, which contains
           the id of the guest to introduce.


        Output keys are :
         - relevant_guest_data: The data of the guest to introduce, a dictionary
           containing their name, drink, and interest.
         - introduce_to: The name (string) of the guest to introduce the guest to.
        """
        # If this is None, we assume we have to infer the guest to introduce
        # based on the named detection
        self._guest_to_introduce = guest_to_introduce
        self._guest_to_introduce_to = guest_to_introduce_to

    def execute(self, userdata: UserData) -> str:
        if self._guest_to_introduce is not None:
            userdata.relevant_guest_data = userdata.guest_data[self._guest_to_introduce]

            introduce_to_reid = userdata.named_guest_detection.name
            if introduce_to_reid not in userdata.guest_data:
                userdata.introduce_to = userdata.guest_data["host"]["name"]
            else:
                userdata.introduce_to = userdata.guest_data[introduce_to_reid]["name"]
        else:
            guest_to_introduce_reid = userdata.named_guest_detection.name
            if guest_to_introduce_reid not in userdata.guest_data:
                userdata.relevant_guest_data = userdata.guest_data["host"]
            else:
                userdata.relevant_guest_data = userdata.guest_data[
                    guest_to_introduce_reid
                ]

            userdata.introduce_to = userdata.guest_data[
                self._guest_to_introduce_to
            ].get("name", "unknown")

        return "succeeded"