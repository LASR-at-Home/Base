"""
State for parsing the transcription of the guests' name and favourite drink, and adding this
to the guest data userdata
"""

from rclpy.node import Node
import smach
from smach import UserData
from smach_ros import ServiceState, RosState
from typing import List, Dict, Any
from HRI.states import SpeechRecovery
from lasr_llm_interfaces.srv import HRITaskQueryLlm

# from tasks.receptionist.src.receptionist.states import SpeechRecovery

class GetNameAndDrink(smach.StateMachine):
    class ParseNameAndDrink(ServiceState):
        def __init__(self, node, task, guest_id):
            super().__init__(node=node, 
                  service_name="/hri_task/query_llm", 
                  service_spec=HRITaskQueryLlm, 
                  request_cb=self.create_req,
                  input_keys=["guest_transcription", "guest_data"],
                  output_keys=["guest_data"],
                  )
            self.task = task
            self.guest_id = guest_id
            
        def create_req(self, userdata, request):
            request = HRITaskQueryLlm(string=userdata.guest_transcription, task=self.task)
            return request
        
        def handle_resp(self, userdata, result):
            userdata.guest_data.update({self.guest_id: {"name": result.name}}) if self.task == "name" else userdata.guest_data.update({self.guest_id: {"drink": result.favoutrite_drink}})

    class PostRecoveryDecision(RosState):
        def __init__(self, node, guest_id: str):
            super().__init__(
                node=node,
                outcomes=["succeeded", "failed", "failed_name", "failed_drink"],
                input_keys=["guest_transcription", "guest_data"],
                output_keys=["guest_data", "guest_transcription"],
            )
            self._guest_id = guest_id

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
        self, node, guest_id: str, last_resort: bool,
    ):

        self._guest_id = guest_id
        self._last_resort = last_resort
        self.node = node
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed", "failed_name", "failed_drink"],
            input_keys=["guest_transcription", "guest_data"],
            output_keys=["guest_data", "guest_transcription"],
        )
        with self:
            super().add(
                "PARSE_NAME",
                self.ParseNameAndDrink(
                    guest_id=self._guest_id, task="name", node=self.node
                ),
                transitions={"succeeded": "PARSE_DRINK", "aborted": "SPEECH_RECOVERY", "preempted": "failed"},
            )
            super().add(
                "PARSE_DRINK",
                self.ParseNameAndDrink(
                    guest_id=self._guest_id, task="drink", node=self.node
                ),
                transitions={"succeeded": "succeeded", "aborted": "SPEECH_RECOVERY", "preempted": "failed"}
            )
            super().add(
                "SPEECH_RECOVERY",
                SpeechRecovery(self._guest_id, self._last_resort),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "POST_RECOVERY_DECISION",
                },
            )
            super().add(
                "POST_RECOVERY_DECISION",
                self.PostRecoveryDecision(
                    guest_id=self._guest_id, node=self.node
                ),
                transitions={
                    "failed": "failed",
                    "failed_name": "failed_name",
                    "failed_drink": "failed_drink",
                },
            )
