"""
State for parsing the transcription of the guests' favourite drink, and adding this
to the guest data userdata
"""

import rospy
import smach

# import llm_utils
from smach import UserData
from typing import List, Dict, Any
from receptionist.states import SpeechRecovery


class GetDrink(smach.StateMachine):
    class ParseDrink(smach.State):
        def __init__(
            self,
            guest_id: str,
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
            self._guest_id = guest_id
            # prior_data: Dict[str, List[str]] = rospy.get_param(param_key)
            # self._possible_drinks = [drink.lower() for drink in prior_data["drinks"]]

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

            extract_fields = llm_utils.extract_fields_llm(
                transcription, ["Drink"]
            )

            if extract_fields["Drink"] != "Unknown":
                userdata.guest_data[self._guest_id]["drink"] = extract_fields["Drink"]
            else:
                userdata.guest_data[self._guest_id]["drink"] = extract_fields["Drink"]
                outcome = "failed"

            return outcome

            # for drink in self._possible_drinks:
            #     if drink in transcription:
            #         userdata.guest_data[self._guest_id]["drink"] = drink
            #         rospy.loginfo(f"Guest Drink identified as: {drink}")
            #         drink_found = True
            #         break

            # if not drink_found:
            #     rospy.loginfo("Drink not found in transcription")
            #     userdata.guest_data[self._guest_id]["drink"] = "unknown"
            #     outcome = "failed"

            # return outcome

    # class PostRecoveryDecision(smach.State):
    #     def __init__(
    #         self,
    #         guest_id: str,
    #         param_key: str = "/receptionist/priors",
    #     ):
    #         smach.State.__init__(
    #             self,
    #             outcomes=["succeeded", "failed"],
    #             input_keys=["guest_transcription", "guest_data"],
    #             output_keys=["guest_data", "guest_transcription"],
    #         )
    #         self._guest_id = guest_id
    #         prior_data: Dict[str, List[str]] = rospy.get_param(param_key)
    #         self._possible_drinks = [drink.lower() for drink in prior_data["drinks"]]

    #     def execute(self, userdata: UserData) -> str:
    #         if userdata.guest_data[self._guest_id]["drink"] == "unknown":
    #             outcome = "failed"
    #         else:
    #             outcome = "succeeded"
    #         return outcome

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
                "PARSE_DRINK",
                self.ParseDrink(guest_id=self._guest_id, param_key=self._param_key),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
            # smach.StateMachine.add(
            #     "SPEECH_RECOVERY",
            #     SpeechRecovery(self._guest_id, self._last_resort, input_type="drink"),
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
            #         "succeeded": "succeeded",
            #     },
            # )
