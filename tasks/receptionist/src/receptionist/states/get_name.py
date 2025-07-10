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


class GetName(smach.StateMachine):
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
                "PARSE_NAME",
                self.ParseName(
                    guest_id=self._guest_id,
                    last_resort=self._last_resort,
                    param_key=self._param_key,
                ),
                transitions={"succeeded": "succeeded", "failed": "RECOVER_NAME"},
            )
            smach.StateMachine.add(
                "RECOVER_NAME",
                self.RecoverName(
                    guest_id=self._guest_id,
                    last_resort=self._last_resort,
                    param_key=self._param_key,
                ),
                transitions={
                    "failed": "SPEECH_RECOVERY_NAME_LLM",
                    "failed_last_resort": "SPEECH_RECOVERY_NAME_LLM_LAST_RESORT",
                    "succeeded": "succeeded",
                },
            )
            smach.StateMachine.add(
                "SPEECH_RECOVERY_NAME_LLM",
                SpeechRecovery(
                    guest_id=self._guest_id,
                    last_resort=self._last_resort,
                    input_type="name",
                    recover_from_llm=True,
                    param_key=self._param_key,
                ),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SPEECH_RECOVERY_NAME_LLM_LAST_RESORT",
                SpeechRecovery(
                    guest_id=self._guest_id,
                    last_resort=self._last_resort,
                    input_type="name",
                    recover_from_llm=True,
                    param_key=self._param_key,
                ),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "SPEECH_RECOVERY_NAME_TRANSCRIPTION_LAST_RESORT",
                },
            )
            smach.StateMachine.add(
                "SPEECH_RECOVERY_NAME_TRANSCRIPTION_LAST_RESORT",
                SpeechRecovery(
                    guest_id=self._guest_id,
                    last_resort=self._last_resort,
                    input_type="name",
                    recover_from_llm=False,
                    param_key=self._param_key,
                ),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )

    class ParseName(smach.State):
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
                "You are a robot acting as a party host. You are tasked with identifying the name "
                "belonging to a guest."
                "You will receive input such as 'my name is john '. Output only the name, "
                " e.g. 'john'. If you can't identify the name, output 'unknown'."
            )
            request.prompt = transcription
            request.max_tokens = 10

            response = self._llm(request)
            # Maxsplit in case the interest is more than one word.
            try:
                if not response:
                    rospy.logwarn("LLM Response Empty")
                    guest["name"] = "unknown"
                    return "failed"
                llm_name = response.output.strip()
            except:
                rospy.logwarn("LLM Response Error")
                guest["name"] = "unknown"
                return "failed"


            # Try to match an exact name from transcription
            name = next(
                (n for n in self._possible_names if n in transcription), llm_name
            )

            # Update guest data
            guest["name"] = name

            rospy.loginfo(
                f"Parsed name: {name} from transcription: {transcription}"
            )

            # Determine outcome
            if name == "unknown":
                return "failed"
            if name not in self._possible_names:
                return "failed"

            return "succeeded"


    class RecoverName(smach.State):
        def __init__(
            self,
            guest_id: str,
            last_resort: bool,
            param_key: str = "/receptionist/priors",
        ):
            smach.State.__init__(
                self,
                outcomes=["failed", "failed_last_resort", "succeeded"],
                input_keys=["guest_transcription", "guest_data"],
                output_keys=["guest_data", "guest_transcription"],
            )
            self._guest_id = guest_id
            prior_data: Dict[str, List[str]] = rospy.get_param(param_key)
            self._possible_names = [name.lower() for name in prior_data["names"]]
            self._last_resort = last_resort

        def execute(self, userdata: UserData) -> str:
            transcription = userdata["guest_transcription"].lower()
            guest = userdata.guest_data[self._guest_id]
            information_found = False

            for key_phrase in self._possible_names:
                if key_phrase in transcription:
                    guest["name"] = key_phrase
                    rospy.loginfo(f"Name identified as: {key_phrase}")
                    information_found = True
                    break
            if not information_found:
                rospy.loginfo(f"Name not found in transcription")
                if self._last_resort:
                    return "failed_last_resort"
                else:
                    return "failed"
            return "succeeded"
