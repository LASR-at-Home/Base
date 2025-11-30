"""
State for parsing the transcription of the guests' name and favourite drink, and adding this
to the guest data userdata
"""

from rclpy.node import Node
import smach
from smach import UserData
from typing import List, Dict, Any
from receptionist.states import SpeechRecovery

# from tasks.receptionist.src.receptionist.states import SpeechRecovery


class GetNameAndDrink(smach.StateMachine):
    class ParseNameAndDrink(smach.State, Node):
        def __init__(self, guest_id: str, param_key: str = "/receptionist/priors"):
            """Parses the transcription of the guests' name and favourite drink.

            Args:
                param_key (str, optional): Name of the parameter that contains the list of
                possible . Defaults to "/receptionist/priors".
            """
            Node.__init__(self, "parse_name_and_drink")
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed", "failed_name", "failed_drink"],
                input_keys=["guest_transcription", "guest_data"],
                output_keys=["guest_data", "guest_transcription"],
            )
            self._guest_id = guest_id
            prior_data: Dict[str, List[str]] = self.get_parameter(param_key).value
            self._possible_names = [name.lower() for name in prior_data["names"]]
            self._possible_drinks = [drink.lower() for drink in prior_data["drinks"]]

        def execute(self, userdata: UserData) -> str:
            """Parses the transcription of the guests' name and favourite drink.

            Args:
                userdata (UserData): State machine userdata assumed to contain a key
                called "guest transcription" with the transcription of the guest's name and
                favourite drink.

            Returns:
                str: state outcome. Updates the userdata with the parsed name and drink, under
                the parameter "guest_data".
            """
            outcome = "succeeded"
            name_found = False
            drink_found = False
            transcription = userdata.guest_transcription.lower()

            transcription = userdata["guest_transcription"].lower()

            for name in self._possible_names:
                if name in transcription:
                    userdata.guest_data[self._guest_id]["name"] = name
                    self.get_logger().info(f"Guest Name identified as: {name}")
                    name_found = True
                    break

            for drink in self._possible_drinks:
                if drink in transcription:
                    userdata.guest_data[self._guest_id]["drink"] = drink
                    self.get_logger().info(f"Guest Drink identified as: {drink}")
                    drink_found = True
                    break

            if not name_found:
                self.get_logger().info("Name not found in transcription")
                userdata.guest_data[self._guest_id]["name"] = "unknown"
                outcome = "failed"
            if not drink_found:
                self.get_logger().info("Drink not found in transcription")
                userdata.guest_data[self._guest_id]["drink"] = "unknown"
                outcome = "failed"

            return outcome

    class PostRecoveryDecision(smach.State, Node):
        def __init__(self, guest_id: str, param_key: str = "/receptionist/priors"):
            Node.__init__(self, "post_recovery_decision")
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed", "failed_name", "failed_drink"],
                input_keys=["guest_transcription", "guest_data"],
                output_keys=["guest_data", "guest_transcription"],
            )
            self._guest_id = guest_id
            prior_data: Dict[str, List[str]] = self.get_parameter(
                param_key
            ).get_parameter_value()  # TODO: check this
            self._possible_names = [name.lower() for name in prior_data["names"]]
            self._possible_drinks = [drink.lower() for drink in prior_data["drinks"]]

        def execute(self, userdata: UserData) -> str:
            if not self._recovery_name_and_drink_required(userdata):
                if userdata.guest_data[self._guest_id]["name"] == "unknown":
                    outcome = "failed_name"
                else:
                    outcome = "failed_drink"
            else:
                outcome = "failed"
            return outcome

        def _recovery_name_and_drink_required(self, userdata: UserData) -> bool:
            """Determine whether both the name and drink requires recovery.

            Returns:
                bool: True if both attributes require recovery.
            """
            if userdata.guest_data[self._guest_id]["name"] == "unknown":
                if userdata.guest_data[self._guest_id]["drink"] == "unknown":
                    return True
            else:
                return False

    def __init__(
        self, guest_id: str, last_resort: bool, param_key: str = "/receptionist/priors"
    ):

        self._guest_id = guest_id
        self._param_key = param_key
        self._last_resort = last_resort

        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed", "failed_name", "failed_drink"],
            input_keys=["guest_transcription", "guest_data"],
            output_keys=["guest_data", "guest_transcription"],
        )
        with self:

            smach.StateMachine.add(
                "PARSE_NAME_AND_DRINK",
                self.ParseNameAndDrink(
                    guest_id=self._guest_id, param_key=self._param_key
                ),
                transitions={"succeeded": "succeeded", "failed": "SPEECH_RECOVERY"},
            )
            smach.StateMachine.add(
                "SPEECH_RECOVERY",
                SpeechRecovery(self._guest_id, self._last_resort),
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
                    "failed_name": "failed_name",
                    "failed_drink": "failed_drink",
                },
            )
