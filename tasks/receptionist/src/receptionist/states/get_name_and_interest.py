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
    Llm,
    LlmRequest,
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
                    guest_id=self._guest_id,last_resort = self._last_resort, param_key=self._param_key
                ),
                transitions={"succeeded": "succeeded", "failed": "failed", "recovery": "SPEECH_RECOVERY"},
            )
            smach.StateMachine.add(
                "SPEECH_RECOVERY",
                self.SpeechRecovery(
                    guest_id=self._guest_id,last_resort = self._last_resort, info_type = "name", param_key = self._param_key
                ),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )

    class ParseNameAndInterest(smach.State):
        def __init__(
            self,
            guest_id: str,
            last_resort: bool, 
            param_key: str = "/receptionist/priors",
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
            self._llm = rospy.ServiceProxy("/lasr_llm/llm", Llm)
            self._llm.wait_for_service()
            self._guest_id = guest_id
            prior_data: Dict[str, List[str]] = rospy.get_param(param_key)
            self._possible_names = [name.lower() for name in prior_data["names"]]
            self._last_resort = last_resort

        def execute(self, userdata: UserData) -> str:
            """Parses the transcription of the guest's name and interest.

            Args:
                userdata (UserData): Must contain key 'guest_transcription' with guest's transcription.

            Returns:
                str: State outcome. Updates 'guest_data' in userdata with parsed name and interest.
            """
            transcription = userdata["guest_transcription"].lower()

            request = LlmRequest()
            request.system_prompt = (
                "You are a robot acting as a party host. You are tasked with identifying the name "
                f"and interest belonging to a guest. The possible names are {','.join(self._possible_names)}. "
                "You will receive input such as 'my name is john and I like robotics'. Output only the name "
                "and interest, e.g. 'john,robotics'. Make sure that the interest is only one or two words. If you can't identify the name or interest, output 'unknown', e.g. 'john,unknown' or 'unknown,robotics'."
                "If you can't identify both the name and the interest, output 'unknown,unknown'"
            )
            request.prompt = transcription
            request.max_tokens = 10

            response = self._llm(request)
            # Maxsplit in case the interest is more than one word.
            try:
                llm_name, interest = response.output.strip().split(",", maxsplit=1)
            except:
                guest["name"] = "unknown"
                guest["interest"] = "unknown"
                return "failed"
            interest_n_words = len(interest.split())
            if interest_n_words > 2:
                interest = interest.split()[
                    :2
                ]  # Take only the first two word of interest
                interest = " ".join(interest)
            interest = interest.strip()

            # Try to match an exact name from transcription
            name = next(
                (n for n in self._possible_names if n in transcription), llm_name
            )

            # Update guest data
            guest = userdata.guest_data[self._guest_id]
            guest["name"] = name
            guest["interest"] = interest

            rospy.loginfo(
                f"Parsed name: {name}, interest: {interest} from transcription: {transcription}"
            )

            # Determine outcome
            if not self._last_resort:
                if name == "unknown" or interest == "unknown":
                    return "failed"
                if name not in self._possible_names:
                    return "failed"

                return "succeeded"
            else:
                if interest == "unknown":
                    request = LlmRequest()
                    request.system_prompt = (
                        "You are a robot acting as a party host. You are tasked with identifying the interest belonging to a guest. "
                        "You will receive input such as 'my name is john and I like robotics'. Output only the "
                        "interest, e.g. 'robotics'. Make sure that the interest is only one or two words. If you can't identify the name or interest, try your best to infer from the input, but only output one or two words."
                    )
                    request.prompt = transcription
                    request.max_tokens = 10

                    response = self._llm(request)
                    # Maxsplit in case the interest is more than one word.
                    interest = response.output.strip()
                        
                    interest_n_words = len(interest.split())
                    if interest_n_words == 0:
                        guest["interest"] = "unknown"
                        return "failed"
                    if interest_n_words > 2:
                        interest = interest.split()[
                            :2
                        ]  # Take only the first two word of interest
                        interest = " ".join(interest)
                    interest = interest.strip()

                    # Update guest data
                    guest = userdata.guest_data[self._guest_id]
                    guest["interest"] = interest

                    rospy.loginfo(
                        f"Parsed interest: {interest} from transcription: {transcription}"
                    )

                if name == "unknown":
                    for key_phrase in self._possible_names:
                        if key_phrase in transcription:
                            guest["name"] = key_phrase
                            rospy.loginfo(
                                f"Name identified as: {key_phrase}"
                            )
                            information_found = True
                            break
                    if not information_found:
                        rospy.loginfo(f"Name not found in transcription")
                        guest["name"] = "unknown"
                        return "recovery"
                    
    


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
        
