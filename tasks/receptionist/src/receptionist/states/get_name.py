"""
State for parsing the transcription of the guests' name and adding this
to the guest data userdata
"""

import rospy
import smach
from smach import UserData
from typing import List, Dict, Any
from receptionist.states import SpeechRecovery


class GetName(smach.StateMachine):
    class ParseName(smach.State):
        def __init__(
            self,
            guest_id: str,
            param_key: str = "/receptionist/priors",
        ):
            """Parses the transcription of the guests' name.

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
            prior_data: Dict[str, List[str]] = rospy.get_param(param_key)
            self._possible_names = [name.lower() for name in prior_data["names"]]

        def execute(self, userdata: UserData) -> str:
            """Parses the transcription of the guests' name.

            Args:
                userdata (UserData): State machine userdata assumed to contain a key
                called "guest transcription" with the transcription of the guest's name.

            Returns:
                str: state outcome. Updates the userdata with the parsed name, under
                the parameter "guest_data".
            """
            outcome = "succeeded"
            name_found = False
            transcription = userdata.guest_transcription.lower()

            transcription = userdata["guest_transcription"].lower()

            for name in self._possible_names:
                if name in transcription:
                    userdata.guest_data[self._guest_id]["name"] = name
                    rospy.loginfo(f"Guest Name identified as: {name}")
                    name_found = True
                    break

            if not name_found:
                rospy.loginfo("Name not found in transcription")
                userdata.guest_data[self._guest_id]["name"] = "unknown"
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

        def execute(self, userdata: UserData) -> str:
            if userdata.guest_data[self._guest_id]["name"] == "unknown":
                outcome = "failed"
            else:
                outcome = "succeeded"
            return outcome

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
                self.ParseName(guest_id=self._guest_id, param_key=self._param_key),
                transitions={"succeeded": "succeeded", "failed": "SPEECH_RECOVERY"},
            )
            smach.StateMachine.add(
                "SPEECH_RECOVERY",
                SpeechRecovery(self._guest_id, self._last_resort, input_type="name"),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "POST_RECOVERY_DECISION",
                },
            )
            smach.StateMachine.add(
                "POST_RECOVERY_DECISION",
                self.PostRecoveryDecision(
                    guest_id=self._guest_id, param_key=self._param_key
                ),
                transitions={
                    "failed": "failed",
                    "succeeded": "succeeded",
                },
            )
