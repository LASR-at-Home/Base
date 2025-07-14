"""
State for parsing the transcription of the guests' favourite drink, and adding this
to the guest data userdata
"""

import rospy
import smach

from smach import UserData
from typing import List, Dict, Any
from receptionist.states import SpeechRecovery
from lasr_llm_msgs.srv import Llm, LlmRequest


class GetDrink(smach.StateMachine):
    def __init__(
        self, guest_id: str, last_resort: bool, param_key: str = "/receptionist/priors"
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
                self.ParseDrink(
                    guest_id=self._guest_id,
                    last_resort=self._last_resort,
                    param_key=self._param_key,
                ),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )

    class ParseDrink(smach.State):
        def __init__(
            self,
            guest_id: str,
            last_resort: bool,
            param_key: str = "/receptionist/priors",
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
            self._llm = rospy.ServiceProxy("/lasr_llm/llm", Llm)
            self._llm.wait_for_service()
            self._guest_id = guest_id
            prior_data: Dict[str, List[str]] = rospy.get_param(param_key)
            self._possible_drinks = [drink.lower() for drink in prior_data["drinks"]]
            self._last_resort = last_resort

        def execute(self, userdata: UserData) -> str:
            """Parses the transcription of the favourite drink.

            Args:
                userdata (UserData): State machine userdata assumed to contain a key
                called "guest transcription" with the transcription of the guest's favourite drink.

            Returns:
                str: state outcome. Updates the userdata with the parsed drink, under
                the parameter "guest_data".
            """
            transcription = userdata["guest_transcription"].lower()
            # Remove punctuation and extra spaces
            transcription = (
                transcription.replace(".", "")
                .replace(",", "")
                .replace("!", "")
                .replace("?", "")
                .strip()
            )

            for drink in self._possible_drinks:
                if drink in transcription:
                    rospy.loginfo(
                        f"Matched drink in transcription: {drink} with transcription: {transcription}"
                    )
                    userdata.guest_data[self._guest_id]["drink"] = drink
                    rospy.loginfo(f"Guest Drink identified as: {drink}")
                    return "succeeded"
            rospy.logwarn(
                f"Could not identify drink from transcription: {transcription}. Retrying..."
            )
            
            request = LlmRequest()
            request.system_prompt = f"You are a robot acting as a party host. You are tasked with identifying the favourite drink belonging to a guest. The possible drinks are {','.join(self._possible_drinks)}. You will receive input such as 'my favourite drink is cola'. Output only the drink, which must exactly match one of the possible drinks. In the previous example this would be 'cola'. If you can't identify the drink, output 'unknown'."
            request.prompt = f"The user says: {transcription}"
            request.max_tokens = 3  # Limit to a single word response
            response = self._llm(request)

            drink = response.output.strip()
            drink_n_words = len(drink.split())
            if drink_n_words > 2:
                drink = drink.split()[
                    :2
                ]  # Take only the first two word of drink
                drink = " ".join(drink)
            drink = drink.strip()
            if "unknown" in drink.lower():
                drink = "unknown"

            if drink.lower() in self._possible_drinks:
                userdata.guest_data[self._guest_id]["drink"] = drink.lower()
            else:
                userdata.guest_data[self._guest_id]["drink"] = "unknown"
                rospy.logwarn(
                    f"Could not identify drink from transcription: {transcription}"
                )
                return "failed"

            rospy.loginfo(f"Guest Drink identified as: {drink}")
            return "succeeded"

    class PostRecoveryDecision(smach.State):
        def __init__(self, guest_id: str, param_key: str = "/receptionist/priors"):
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
