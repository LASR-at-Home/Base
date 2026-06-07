"""
State for parsing the transcription of the guests' name and favourite drink, and adding this
to the guest data userdata
"""

from rclpy.node import Node
import yasmin
import yasmin_ros
from typing import List, Dict, Any
from HRI.states import SpeechRecovery
from lasr_llm_interfaces.srv import HRITaskQueryLlm

# from tasks.receptionist.src.receptionist.states import SpeechRecovery


class GetNameAndDrink(yasmin.StateMachine):
    class ParseNameAndDrink(yasmin_ros.ServiceState):
        def __init__(self, task, guest_id):
            super().__init__(
                srv_name="/hri_task/query_llm",
                srv_type=HRITaskQueryLlm,
                create_request_handler=self._create_req,
                response_handler=self._handle_resp,
            )

            self.add_input_key("guest_transcription")
            self.add_input_key("guest_data")
            self.add_output_key("guest_data")

            self.task = task
            self.guest_id = guest_id

        def _create_req(self, blackboard):
            request = HRITaskQueryLlm.Request(
                string=blackboard['guest_transcription'], task=self.task
            )

            return request

        def _handle_resp(self, blackboard, result):
            (
                blackboard["guest_data"].update({self.guest_id: {"name": result.name}})
                if self.task == "name"
                else blackboard["guest_data"].update(
                    {self.guest_id: {"drink": result.favoutrite_drink}}
                )
            )

            return "succeeded"

    class PostRecoveryDecision(yasmin.State):
        def __init__(self, guest_id: str):
            super().__init__(
                outcomes=["succeeded", "failed", "failed_name", "failed_drink"],
            )

            self.add_input_key("guest_transcription")
            self.add_input_key("guest_data")

            self.add_output_key("guest_transcription")
            self.add_output_key("guest_data")

            self._guest_id = guest_id

        def execute(self, blackboard) -> str:
            if not self._recovery_name_and_drink_required(blackboard):
                if blackboard["guest_data"][self._guest_id]["name"] == "unknown":
                    outcome = "failed_name"
                else:
                    outcome = "failed_drink"
            else:
                outcome = "failed"
            return outcome

        def _recovery_name_and_drink_required(self, blackboard) -> bool:
            """Determine whether both the name and drink requires recovery.

            Returns:
                bool: True if both attributes require recovery.
            """

            return (
                blackboard["guest_data"][self._guest_id]["name"] == "unknown"
                and blackboard["guest_data"][self._guest_id]["drink"] == "unknown"
            )

    def __init__(
        self,
        guest_id: str,
        last_resort: bool,
    ):
        super().__init__(
            outcomes=["succeeded", "failed", "failed_name", "failed_drink"],
            handle_sigint=True,
        )

        self.add_input_key("guest_transcription")
        self.add_input_key("guest_data")
        self.add_output_key("guest_transcription")
        self.add_output_key("guest_data")

        self.add_state(
            "PARSE_NAME",
            self.ParseNameAndDrink(guest_id=guest_id, task="name"),
            transitions={
                "succeeded": "PARSE_DRINK",
                "aborted": "SPEECH_RECOVERY",
            },
        )
        self.add_state(
            "PARSE_DRINK",
            self.ParseNameAndDrink(guest_id=guest_id, task="drink"),
            transitions={
                "succeeded": "succeeded",
                "aborted": "SPEECH_RECOVERY",
            },
        )
        self.add_state(
            "SPEECH_RECOVERY",
            SpeechRecovery(guest_id, last_resort),
            transitions={
                "succeeded": "succeeded",
                "failed": "POST_RECOVERY_DECISION",
            },
        )
        self.add_state(
            "POST_RECOVERY_DECISION",
            self.PostRecoveryDecision(guest_id=guest_id),
            transitions={
                "failed": "failed",
                "failed_name": "failed_name",
                "failed_drink": "failed_drink",
            },
        )
