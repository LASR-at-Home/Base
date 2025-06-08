"""
State for parsing the transcription of the guests' name and favourite interest, and adding this
to the guest data userdata
"""

import rospy
import smach
from smach import UserData
from typing import List, Dict, Any
from receptionist.states import SpeechRecovery
from lasr_llm_msgs.srv import (
    ReceptionistQueryLlm,
    ReceptionistQueryLlmRequest,
    ReceptionistQueryLlmResponse,
)


class GetNameAndInterest(smach.StateMachine):
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

    class ParseNameAndInterest(smach.State):
        def __init__(
            self,
            guest_id: str,
            param_key: str = "/receptionist/priors",
            llm_service: str = "/receptionist/query_llm",
        ):
            """Parses the transcription of the guests' name and interest.

            Args:
                param_key (str, optional): Name of the parameter that contains the list of
                prior knowledge . Defaults to "/receptionist/priors".
            """
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["guest_transcription", "guest_data"],
                output_keys=["guest_data", "guest_transcription"],
            )
            rospy.wait_for_service(llm_service)
            self._llm_service_client = rospy.ServiceProxy(
                llm_service, ReceptionistQueryLlm
            )
            self._guest_id = guest_id
            prior_data: Dict[str, List[str]] = rospy.get_param(param_key)
            self._possible_names = [name.lower() for name in prior_data["names"]]

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

            request = ReceptionistQueryLlmRequest()
            request.llm_input = transcription
            request.task = "name_and_interest"

            response = self._llm_service_client(request).response

            name_found = False
            # Check if the name is in the transcription
            for possible_name in self._possible_names:
                if possible_name in transcription:
                    name = possible_name
                    name_found = True
                    break

            if not name_found:
                name = response.name.lower() if response.name else "unknown"
            interest = response.interests.lower() if response.interests else None

            if name is "unknown" or interest is None:
                outcome = "failed"
            elif name not in self._possible_names:
                userdata.guest_data[self._guest_id]["interest"] = (
                    interest if interest else "unknown"
                )
                outcome = "failed"

            userdata.guest_data[self._guest_id]["name"] = name if name else "unknown"
            userdata.guest_data[self._guest_id]["interest"] = (
                interest if interest else "unknown"
            )

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
            return False
