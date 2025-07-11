"""
State for parsing the transcription of the guests' name and favourite interest, and adding this
to the guest data userdata
"""

import rospy
import smach
from smach import UserData
from typing import List, Dict, Any
from receptionist.states import SpeechRecovery
from lasr_llm_msgs.srv import Llm, LlmRequest


class GetNameAndInterest(smach.StateMachine):
    def __init__(
        self, guest_id: str, last_resort: bool, param_key: str = "/receptionist/priors"
    ):

        self._guest_id = guest_id
        self._param_key = param_key
        self._last_resort = last_resort

        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed", "failed_interest", "failed_name"],
            input_keys=["guest_transcription", "guest_data"],
            output_keys=["guest_data", "guest_transcription"],
        )
        with self:
            smach.StateMachine.add(
                "PARSE_NAME_AND_INTEREST",
                self.ParseNameAndInterest(
                    guest_id=self._guest_id,
                    last_resort=self._last_resort,
                    param_key=self._param_key,
                ),
                transitions={"succeeded": "succeeded", "failed": "RECOVERY_DECISION"},
            )
            smach.StateMachine.add(
                "RECOVERY_DECISION",
                self.RecoveryDecision(
                    guest_id=self._guest_id,
                    last_resort=self._last_resort,
                    param_key=self._param_key,
                ),
                transitions={
                    "failed_name": "RECOVER_NAME",
                    "failed_interest": "RECOVER_INTEREST",
                    "failed": "RECOVER_BOTH_INTEREST",
                },
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
                    "succeeded": "POST_RECOVERY_DECISION",
                    "failed": "POST_RECOVERY_DECISION",
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
                    "succeeded": "POST_RECOVERY_DECISION",
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
                    "succeeded": "POST_RECOVERY_DECISION",
                    "failed": "POST_RECOVERY_DECISION",
                },
            )
            smach.StateMachine.add(
                "RECOVER_INTEREST",
                self.RecoverInterest(
                    guest_id=self._guest_id,
                    last_resort=self._last_resort,
                    param_key=self._param_key,
                ),
                transitions={
                    "failed": "POST_RECOVERY_DECISION",
                    "succeeded": "POST_RECOVERY_DECISION",
                },
            )
            smach.StateMachine.add(
                "RECOVER_BOTH_INTEREST",
                self.RecoverInterest(
                    guest_id=self._guest_id,
                    last_resort=self._last_resort,
                    param_key=self._param_key,
                ),
                transitions={"failed": "RECOVER_NAME", "succeeded": "RECOVER_NAME"},
            )
            smach.StateMachine.add(
                "POST_RECOVERY_DECISION",
                self.PostRecoveryDecision(
                    guest_id=self._guest_id,
                    last_resort=self._last_resort,
                    param_key=self._param_key,
                ),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                    "failed_name": "failed_name",
                    "failed_interest": "failed_interest",
                },
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

            guest = userdata.guest_data[self._guest_id]

            request = LlmRequest()
            request.system_prompt = (
                "You are a robot acting as a party host. You are tasked with identifying the name "
                "and interest belonging to a guest."
                "You will receive input such as 'my name is john and I like robotics'. Output only the name "
                "and interest, e.g. 'john,robotics'. Make sure that the interest is only one or two words. If you can't identify the name or interest, output 'unknown', e.g. 'john,unknown' or 'unknown,robotics'."
                "If you can't identify both the name and the interest, output 'unknown,unknown'"
            )
            request.prompt = transcription
            request.max_tokens = 10

            response = self._llm(request)
            # Maxsplit in case the interest is more than one word.
            try:
                if not response:
                    rospy.logwarn("LLM Response Empty")
                    guest["name"] = "unknown"
                    guest["interest"] = "unknown"
                    return "failed"
                llm_name, interest = response.output.strip().split(",", maxsplit=1)
            except:
                rospy.logwarn("LLM Response Error")
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
            if "unknown" in interest.lower():
                interest = "unknown"

            # Try to match an exact name from transcription
            name = next(
                (n for n in self._possible_names if n in transcription), llm_name
            )

            # Update guest data
            guest["name"] = name
            guest["interest"] = interest

            rospy.loginfo(
                f"Parsed name: {name}, interest: {interest} from transcription: {transcription}"
            )

            # Determine outcome
            if name == "unknown" or interest == "unknown":
                return "failed"
            if name not in self._possible_names:
                return "failed"

            return "succeeded"

    class RecoveryDecision(smach.State):
        def __init__(
            self,
            guest_id: str,
            last_resort: bool,
            param_key: str = "/receptionist/priors",
        ):
            smach.State.__init__(
                self,
                outcomes=["failed_name", "failed_interest", "failed"],
                input_keys=["guest_transcription", "guest_data"],
                output_keys=["guest_data", "guest_transcription"],
            )
            self._guest_id = guest_id
            prior_data: Dict[str, List[str]] = rospy.get_param(param_key)
            self._possible_names = [name.lower() for name in prior_data["names"]]
            self._last_resort = last_resort

        def execute(self, userdata: UserData) -> str:
            if not self._recovery_name_and_interest_required(userdata):
                if (
                    userdata.guest_data[self._guest_id]["name"] == "unknown"
                    or userdata.guest_data[self._guest_id]["name"]
                    not in self._possible_names
                ):
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
            if (
                userdata.guest_data[self._guest_id]["name"] == "unknown"
                or userdata.guest_data[self._guest_id]["name"]
                not in self._possible_names
            ):
                if userdata.guest_data[self._guest_id]["interest"] == "unknown":
                    return True
            return False

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

    class PostRecoveryDecision(smach.State):
        def __init__(
            self,
            guest_id: str,
            last_resort: bool,
            param_key: str = "/receptionist/priors",
        ):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed", "failed_name", "failed_interest"],
                input_keys=["guest_transcription", "guest_data"],
                output_keys=["guest_data", "guest_transcription"],
            )
            self._guest_id = guest_id
            prior_data: Dict[str, List[str]] = rospy.get_param(param_key)
            self._possible_names = [name.lower() for name in prior_data["names"]]
            self._last_resort = last_resort

        def execute(self, userdata: UserData) -> str:
            if not self._recovery_name_and_interest_required(userdata):
                if (
                    userdata.guest_data[self._guest_id]["name"] == "unknown"
                    or userdata.guest_data[self._guest_id]["name"]
                    not in self._possible_names
                ):
                    outcome = "failed_name"
                elif userdata.guest_data[self._guest_id]["interest"] == "unknown":
                    outcome = "failed_interest"
                else:
                    outcome = "succeeded"
            else:
                outcome = "failed"
            return outcome

        def _recovery_name_and_interest_required(self, userdata: UserData) -> bool:
            """Determine whether both the name and interest requires recovery.

            Returns:
                bool: True if both attributes require recovery.
            """
            if (
                userdata.guest_data[self._guest_id]["name"] == "unknown"
                or userdata.guest_data[self._guest_id]["name"]
                not in self._possible_names
            ):
                if userdata.guest_data[self._guest_id]["interest"] == "unknown":
                    return True
            return False
