"""
State for parsing the transcription of the guests' name, and adding this
to the guest data userdata
"""

import rospy
import smach
from smach import UserData
from typing import List, Dict, Any


class ParseName(smach.State):
    def __init__(
        self,
        guest_id: str,
        param_key: str = "/priors",
    ):
        """Parses the transcription of the guests' name and favourite drink.

        Args:
            param_key (str, optional): Name of the parameter that contains the list of
            possible . Defaults to "receptionist/priors".
        """
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_transcription", "guest_data"],
            output_keys=["guest data", "guest_transcription"],
        )
        self._guest_id = guest_id
        prior_data: Dict[str, List[str]] = rospy.get_param(param_key)
        self._possible_names = [name.lower() for name in prior_data["names"]]
        self._possible_drinks = [drink.lower() for drink in prior_data["drinks"]]

    def execute(self, userdata: UserData) -> str:
        """Handle the recovery behaviour and parse in the guest name again.

        Args:
            userdata (UserData): State machine userdata assumed to contain a key
            called "guest transcription" with the transcription of the guest's name and
            favourite drink.

        Returns:
            str: state outcome. Updates the userdata with the parsed name and drink, under
            the parameter "guest data".
        """
        outcome = "succeeded"
        name_found = False
        print(userdata)
        print(type(userdata.guest_transcription))
        transcription = userdata.guest_transcription.lower()

        transcription = userdata["guest_transcription"].lower()

        for name in self._possible_names:
            if name in transcription:
                userdata.guest_data[self._guest_id]["name"] = name
                rospy.loginfo(f"Guest Name identified as: {name}")
                name_found = True
                break
        if not name_found:
            rospy.loginfo("Name not found in transcription")
            userdata.guest_data[self._guest_id]["name"] = "unknown"
            outcome = "failed"

        return outcome
    
