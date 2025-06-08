"""
State for parsing the transcription of the guests' favourite drink, and adding this
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


class GetDrink(smach.StateMachine):
    def __init__(
        self,
        guest_id: str,
        last_resort: bool,
        param_key: str = "/receptionist/priors",
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_transcription", "guest_data"],
            output_keys=["guest_data", "guest_transcription"],
        )

        self._guest_id = guest_id
        self._param_key = param_key
        self._last_resort = last_resort
        with self:
            smach.StateMachine.add(
                "PARSE_DRINK",
                self.ParseDrink(guest_id=self._guest_id, param_key=self._param_key),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
    class ParseDrink(smach.State):
        def __init__(
            self,
            guest_id: str,
            param_key: str = "/receptionist/priors",
            llm_service: str = "/receptionist/query_llm",
        ):
            """Parses the transcription of the guests' favourite drink.

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
            self._llm_client = rospy.ServiceProxy(llm_service, ReceptionistQueryLlm)
            self._llm_client.wait_for_service()
            self._guest_id = guest_id
            prior_data: Dict[str, List[str]] = rospy.get_param(param_key)
            self._possible_drinks = [drink.lower() for drink in prior_data["drinks"]]

        def execute(self, userdata: UserData) -> str:
            """Parses the transcription of the favourite drink.

            Args:
                userdata (UserData): State machine userdata assumed to contain a key
                called "guest transcription" with the transcription of the guest's favourite drink.

            Returns:
                str: state outcome. Updates the userdata with the parsed drink, under
                the parameter "guest_data".
            """
            outcome = "succeeded"
            # drink_found = False
            transcription = userdata.guest_transcription.lower()
            transcription = userdata["guest_transcription"].lower()

            # extract_fields = llm_utils.extract_fields_llm(
            #     transcription, ["Favourite drink"]
            # )

            # if extract_fields["drink"] != "Unknown":
            #     userdata.guest_data[self._guest_id]["drink"] = extract_fields["drink"]
            # else:
            #     userdata.guest_data[self._guest_id]["interest"] = extract_fields[
            #         "interest"
            #     ]
            #     outcome = "failed"

            # return outcome
            drink_found = False
            for drink in self._possible_drinks:
                if drink in transcription:
                    userdata.guest_data[self._guest_id]["drink"] = drink
                    rospy.loginfo(f"Guest Drink identified as: {drink}")
                    drink_found = True
                    break

            if not drink_found:
                request = ReceptionistQueryLlmRequest()
                request.llm_input = transcription
                request.task = "drink"
                response: ReceptionistQueryLlmResponse = self._llm_client(
                    request
                ).response
                drink = response.favourite_drink
                if drink is not None and drink.lower() in self._possible_drinks:
                    userdata.guest_data[self._guest_id]["drink"] = drink.lower()
                    rospy.loginfo(f"Guest Drink identified as: {drink}")
                else:
                    userdata.guest_data[self._guest_id]["drink"] = "unknown"
                    rospy.logwarn(
                        f"Could not identify drink from transcription: {transcription}"
                    )
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
            self._possible_drinks = [drink.lower() for drink in prior_data["drinks"]]

        def execute(self, userdata: UserData) -> str:
            if userdata.guest_data[self._guest_id]["drink"] == "unknown":
                outcome = "failed"
            else:
                outcome = "succeeded"
            return outcome
