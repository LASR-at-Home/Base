"""
State for parsing the transcription of the guests' interest, and adding this
to the guest data userdata
"""

import rospy
import smach
from smach import UserData
from typing import List, Dict, Any
from storing_groceries.states import SpeechRecovery


class GetInterest(smach.StateMachine):
    class ParseInterest(smach.State):
        def __init__(
            self,
            guest_id: str,
            param_key: str = "/receptionist/priors",
        ):
            """Parses the transcription of the guests' .

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
            self._possible_interests = [interest.lower() for interest in prior_data["interests"]]

        def execute(self, userdata: UserData) -> str:
            """Parses the transcription of the guests' interest.

            Args:
                userdata (UserData): State machine userdata assumed to contain a key
                called "guest transcription" with the transcription of the guest's interest.

            Returns:
                str: state outcome. Updates the userdata with the parsed interest, under
                the parameter "guest_data".
            """
            outcome = "succeeded"
            interest_found = False
            transcription = userdata.guest_transcription.lower()

            transcription = userdata["guest_transcription"].lower()

            for interest in self._possible_interests:
                if interest in transcription:
                    userdata.guest_data[self._guest_id]["interest"] = interest
                    rospy.loginfo(f"Guest Interest identified as: {interest}")
                    interest_found = True
                    break

            if not interest_found:
                rospy.loginfo("Interest not found in transcription")
                userdata.guest_data[self._guest_id]["interest"] = "unknown"
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
            self._possible_interests = [interest.lower() for interest in prior_data["interests"]]

        def execute(self, userdata: UserData) -> str:
            if userdata.guest_data[self._guest_id]["interest"] == "unknown":
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
                "PARSE_INTEREST",
                self.ParseInterest(
                    guest_id=self._guest_id, param_key=self._param_key
                ),
                transitions={"succeeded": "succeeded", "failed": "SPEECH_RECOVERY"},
            )
            smach.StateMachine.add(
                "SPEECH_RECOVERY",
                SpeechRecovery(self._guest_id, self._last_resort, input_type="interest"),
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
