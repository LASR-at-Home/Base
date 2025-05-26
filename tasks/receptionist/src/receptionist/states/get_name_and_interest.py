"""
State for parsing the transcription of the guests' name and favourite interest, and adding this
to the guest data userdata
"""

import rospy
import smach
from smach import UserData
from typing import List, Dict, Any
from receptionist.states import SpeechRecovery

# import llm_utils


class GetNameAndInterest(smach.StateMachine):
    class ParseNameAndInterest(smach.State):
        def __init__(
            self,
            guest_id: str,
            param_key: str = "/receptionist/priors",
        ):
            """Parses the transcription of the guests' name and interest.

            Args:
                param_key (str, optional): Name of the parameter that contains the list of
                possible . Defaults to "/receptionist/priors".
            """
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["guest_transcription", "guest_data"],
                output_keys=["guest_data", "guest_transcription"],
            )
            self._guest_id = guest_id
            # prior_data: Dict[str, List[str]] = rospy.get_param(param_key)
            # self._possible_names = [name.lower() for name in prior_data["names"]]
            # self._possible_interests = [interest.lower() for interest in prior_data["interest"]]

        def execute(self, userdata: UserData) -> str:
            """Parses the transcription of the guests' name and interest.

            Args:
                userdata (UserData): State machine userdata assumed to contain a key
                called "guest transcription" with the transcription of the guest's name and
                interest.

            Returns:
                str: state outcome. Updates the userdata with the parsed name and interest, under
                the parameter "guest_data".
            """
            outcome = "succeeded"
            transcription = userdata.guest_transcription.lower()
            transcription = userdata["guest_transcription"].lower()

            # extract_fields = llm_utils.extract_fields_llm(
            #     transcription, ["Name", "Interests"]
            # )
            extract_fields = {"name": "Charlie", "interest": "sports"}

            if extract_fields["name"] != "Unknown":
                userdata.guest_data[self._guest_id]["name"] = extract_fields["name"]
            else:
                userdata.guest_data[self._guest_id]["interest"] = extract_fields[
                    "interest"
                ]
                outcome = "failed"

            if extract_fields["interest"] != "Unknown":
                userdata.guest_data[self._guest_id]["interest"] = extract_fields[
                    "interest"
                ]
            else:
                userdata.guest_data[self._guest_id]["interest"] = extract_fields[
                    "interest"
                ]
                outcome = "failed"

            return outcome

    class PostRecoveryDecision(smach.State):
        def __init__(
            self,
            guest_id: str,
            param_key: str = "/receptionist/priors",
        ):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["guest_transcription", "guest_data"],
                output_keys=["guest_data", "guest_transcription"],
            )
            self._guest_id = guest_id
            prior_data: Dict[str, List[str]] = rospy.get_param(param_key)
            self._possible_names = [name.lower() for name in prior_data["names"]]
            self._possible_interests = [
                interest.lower() for interest in prior_data["interests"]
            ]

        def execute(self, userdata: UserData) -> str:
            if not self._recovery_name_and_interest_required(userdata):
                if userdata.guest_data[self._guest_id]["name"] == "unknown":
                    outcome = "failed_name"
                else:
                    outcome = "failed_interest"
            else:
                outcome = "failed"
            return outcome

        def _recovery_name_and_interest_required(self, userdata: UserData) -> bool:
            """Determine whether both the name and interest requires recovery.

            Returns:
                bool: True if both attributes require recovery.
            """
            if userdata.guest_data[self._guest_id]["name"] == "unknown":
                if userdata.guest_data[self._guest_id]["interest"] == "unknown":
                    return True
            else:
                return False

    def __init__(
        self,
        guest_id: str,
        last_resort: bool,
        param_key: str = "/receptionist/priors",
    ):

        self._guest_id = guest_id
        self._param_key = param_key
        self._last_resort = last_resort

        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_transcription", "guest_data"],
            output_keys=["guest_data", "guest_transcription"],
        )
        with self:

            smach.StateMachine.add(
                "PARSE_NAME_AND_INTEREST",
                self.ParseNameAndInterest(
                    guest_id=self._guest_id, param_key=self._param_key
                ),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
            # smach.StateMachine.add(
            #     "SPEECH_RECOVERY",
            #     SpeechRecovery(self._guest_id, self._last_resort),
            #     transitions={
            #         "succeeded": "succeeded",
            #         "failed": "POST_RECOVERY_DECISION",
            #     },
            # )
            # smach.StateMachine.add(
            #     "POST_RECOVERY_DECISION",
            #     self.PostRecoveryDecision(
            #         guest_id=self._guest_id, param_key=self._param_key
            #     ),
            #     transitions={
            #         "failed": "failed",
            #         "failed_name": "failed_name",
            #         "failed_interest": "failed_interest",
            #     },
            # )
