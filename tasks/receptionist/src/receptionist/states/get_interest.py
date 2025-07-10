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


class GetInterest(smach.StateMachine):
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
                "PARSE_INTEREST",
                self.ParseInterest(
                    guest_id=self._guest_id,
                    last_resort=self._last_resort,
                    param_key=self._param_key,
                ),
                transitions={"succeeded": "succeeded", "failed": "RECOVER_INTEREST"},
            )
            smach.StateMachine.add(
                "RECOVER_INTEREST",
                self.RecoverInterest(
                    guest_id=self._guest_id,
                    last_resort=self._last_resort,
                    param_key=self._param_key,
                ),
                transitions={
                    "failed": "failed",
                    "succeeded": "succeeded",
                },
            )

    class ParseInterest(smach.State):
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

            guest = userdata.guest_data[self._guest_id]

            request = LlmRequest()
            request.system_prompt = (
                "You are a robot acting as a party host. You are tasked with identifying "
                "interest belonging to a guest."
                "You will receive input such as 'I like robotics' or 'My interest is robotics'. Output only the"
                "interest, e.g. 'robotics'. Make sure that the interest is only one or two words. If you can't identify the interest, output 'unknown'."
            )
            request.prompt = transcription
            request.max_tokens = 10

            response = self._llm(request)
            # Maxsplit in case the interest is more than one word.
            try:
                if not response:
                    rospy.logwarn("LLM Response Empty")
                    guest["interest"] = "unknown"
                    return "failed"
                interest = response.output.strip()
            except:
                rospy.logwarn("LLM Response Error")
                guest["interest"] = "unknown"
                return "failed"

            interest_n_words = len(interest.split())
            if interest_n_words > 2:
                interest = interest.split()[
                    :2
                ]  # Take only the first two word of interest
                interest = " ".join(interest)
            interest = interest.strip()
            if "unknown" in interest.lower() or interest == "":
                interest = "unknown"

            guest["interest"] = interest

            rospy.loginfo(
                f"Parsed  interest: {interest} from transcription: {transcription}"
            )

            # Determine outcome
            if interest == "unknown":
                return "failed"
            return "succeeded"

    class RecoverInterest(smach.State):
        def __init__(
            self,
            guest_id: str,
            last_resort: bool,
            param_key: str = "/receptionist/priors",
        ):
            smach.State.__init__(
                self,
                outcomes=["failed", "succeeded"],
                input_keys=["guest_transcription", "guest_data"],
                output_keys=["guest_data", "guest_transcription"],
            )
            self._guest_id = guest_id
            prior_data: Dict[str, List[str]] = rospy.get_param(param_key)
            self._possible_names = [name.lower() for name in prior_data["names"]]
            self._last_resort = last_resort

        def execute(self, userdata: UserData) -> str:
            if not self._last_resort:
                return "failed"

            guest = userdata.guest_data[self._guest_id]
            guest["interest"] = "technology"

            rospy.loginfo(f"Resort to recovering interest as technology")
            return "succeeded"

