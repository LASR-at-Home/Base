"""
State for parsing the transcription of the guests' name and favourite drink, and adding this
to the guest data userdata
"""

import rospy
import smach
from smach import UserData
from typing import List, Dict, Any


class ParseNameAndDrink(smach.State):
    def __init__(
        self,
        param_key: str = "priors",
        outcomes: List[str] = ["succeeded", "failed"],
        input_keys: List[str] = ["guest_transcription", "guest_id", "guest_data"],
        output_keys: List[str] = ["guest data"],
    ):
        """Parses the transcription of the guests' name and favourite drink.

        Args:
            param_key (str, optional): Name of the parameter that contains the list of
            possible . Defaults to "priors".
        """
        super().__init__(
            outcomes=outcomes,
            input_keys=input_keys,
            output_keys=output_keys,
        )
        prior_data: Dict[str, List[str]] = rospy.get_param(param_key)
        self._possible_names = prior_data["names"]
        self._possible_drinks = prior_data["drinks"]

    def execute(self, userdata: UserData) -> str:
        """Parses the transcription of the guests' name and favourite drink.

        Args:
            userdata (UserData): State machine userdata assumed to contain a key
            called "guest transcription" with the transcription of the guest's name and
            favourite drink.

        Returns:
            str: state outcome. Updates the userdata with the parsed name and drink, under
            the parameter "guest data".
        """

        outcome = "succeeded"
        guest_id = userdata["guest_id"]
        name_found = False
        drink_found = False

        for name in self._possible_names:
            if name in userdata["guest_transcription"]:
                userdata["guest_data"][guest_id]["name"] = name
                rospy.loginfo(f"Guest Name identified as: {name}")
                name_found = True
                break

        for drink in self._possible_drinks:
            if drink in userdata["guest_transcription"]:
                userdata["guest_data"][guest_id]["drink"] = drink
                rospy.loginfo(f"Guest Drink identified as: {drink}")
                drink_found = True
                break

        if not name_found:
            rospy.loginfo("Name not found in transcription")
            outcome = "failed"
        if not drink_found:
            rospy.loginfo("Drink not found in transcription")
            outcome = "failed"

        return outcome
