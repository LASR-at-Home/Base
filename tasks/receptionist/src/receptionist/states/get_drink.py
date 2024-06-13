"""
State for parsing the transcription of the guests' favourite drink, and adding this
to the guest data userdata
"""

import rospy
import smach
from smach import UserData
from typing import List, Dict, Any


class ParseDrink(smach.State):
    def __init__(
        self,
        guest_id: str,
        # param_key: str = "/priors",
        param_key: str = "/receptionist/priors",
    ):
        """Parses the transcription of the guests' favourite drink.

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
        self._possible_drinks = [drink.lower() for drink in prior_data["drinks"]]

    def execute(self, userdata: UserData) -> str:
        """Handle the recovery behaviour and attempts to parse the guest's drink.

        Args:
            userdata (UserData): State machine userdata assumed to contain a key
            called "guest transcription" with the transcription of the guest's name and
            favourite drink.

        Returns:
            str: state outcome. Updates the userdata with the parsed drink, under
            the parameter "guest data".
        """
        outcome = "succeeded"
        drink_found = False
        transcription = userdata.guest_transcription.lower()

        transcription = userdata["guest_transcription"].lower()

        for drink in self._possible_drinks:
            print(self._possible_drinks)
            print(transcription)
            if drink in transcription:
                userdata.guest_data[self._guest_id]["drink"] = drink
                rospy.loginfo(f"Guest Drink identified as: {drink}")
                drink_found = True
                break
        if not drink_found:
            rospy.loginfo("Drink not found in transcription")
            userdata.guest_data[self._guest_id]["drink"] = "unknown"
            outcome = "failed"

        return outcome
