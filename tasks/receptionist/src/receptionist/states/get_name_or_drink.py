"""
State for parsing the transcription of the guests' information (favourite drink or name), and adding this
to the guest data userdata
"""

import rospy
import smach
from smach import UserData
from typing import List, Dict, Any


class ParseTranscribedInfo(smach.State):
    def __init__(
        self,
        guest_id: str,
        info_type: str,
        param_key: str = "/receptionist/priors",
    ):
        """Parses the transcription of the guests' information.

        Args:
            guest_id (str): ID of the guest (identifying the guest)
            info_type (str): The type of information to try and extract useful information
            (drink or name)
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
        self._type = info_type
        prior_data: Dict[str, List[str]] = rospy.get_param(param_key)
        possible_drinks = [drink.lower() for drink in prior_data["drinks"]]
        possible_names = [name.lower() for name in prior_data["names"]]
        self._possible_information = {"drink": possible_drinks, "name": possible_names}[
            self._type
        ]

    def execute(self, userdata: UserData) -> str:
        """Parse the guest's information.

        Args:
            userdata (UserData): State machine userdata assumed to contain a key
            called "guest transcription" with the transcription of the guest's name or
            favourite drink or both.

        Returns:
            str: state outcome. Updates the userdata with the parsed information (drink or name), under
            the parameter "guest data".
        """
        outcome = "succeeded"
        information_found = False
        transcription = userdata.guest_transcription.lower()

        transcription = userdata["guest_transcription"].lower()

        for key_phrase in self._possible_information:
            print(self._possible_information)
            if key_phrase in transcription:
                userdata.guest_data[self._guest_id][self._type] = key_phrase
                rospy.loginfo(f"Guest {self._type} identified as: {key_phrase}")
                information_found = True
                break
        if not information_found:
            rospy.loginfo(f"{self._type} not found in transcription")
            userdata.guest_data[self._guest_id][self._type] = "unknown"
            outcome = "failed"

        return outcome
