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
import rclpy

# from tasks.receptionist.src.receptionist.states import SpeechRecovery

from lasr_skills import AskAndListen


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
            self.add_output_key("placeholders")

            self.task = task
            self.guest_id = guest_id

        def _create_req(self, blackboard):
            request = HRITaskQueryLlm.Request(
                llm_input=blackboard["guest_transcription"], task=self.task
            )

            return request

        def _handle_resp(self, blackboard, result):
            result = result.response
            if result.name == "" and result.favourite_drink == "":
                return "aborted"
            blackboard["guest_data"][self.guest_id][self.task] = (
                result.name if self.task == "name" else result.favourite_drink
            )

            if self.task == "name":
                blackboard["placeholders"] = result.name

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
                if blackboard["guest_data"][self._guest_id]["name"] == "":
                    outcome = "failed_name"
                    blackboard["guest_data"][self._guest_id]["name"] = "John"
                else:
                    blackboard["guest_data"][self._guest_id]["drink"] = "Coke"
                    outcome = "failed_drink"
            else:
                blackboard["guest_data"][self._guest_id]["name"] = "John"
                blackboard["guest_data"][self._guest_id]["drink"] = "Coke"
                outcome = "failed"

            blackboard["placeholders"] = "John"
            yasmin.YASMIN_LOG_INFO(str(blackboard["guest_data"]))
            return outcome

        def _recovery_name_and_drink_required(self, blackboard) -> bool:
            """Determine whether both the name and drink requires recovery.

            Returns:
                bool: True if both attributes require recovery.
            """

            return (
                blackboard["guest_data"][self._guest_id]["name"] == ""
                and blackboard["guest_data"][self._guest_id]["drink"] == ""
            )

    def __init__(
        self,
        guest_id: str,
        last_resort: bool,
    ):
        super().__init__(outcomes=["succeeded", "failed"])

        self.flag = True

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
        # self.add_state(
        #     "SPEECH_RECOVERY",
        #     SpeechRecovery(guest_id, last_resort),
        #     transitions={
        #         "succeeded": "succeeded",
        #         "failed": "POST_RECOVERY_DECISION",
        #     },
        # )

        self.add_state(
            "SPEECH_RECOVERY",
            self.PostRecoveryDecision(guest_id=guest_id),
            transitions={
                "failed": "CHECK_REPEAT",
                "failed_name": "CHECK_REPEAT",
                "failed_drink": "CHECK_REPEAT",
            },
        )

        self.add_state(
            "CHECK_REPEAT",
            yasmin.CbState(outcomes=["succeeded", "continue"], callback=self.check),
            transitions={"succeeded": "succeeded", "continue": "REPEAT_ASK_GUEST"},
        )

        self.add_state(
            "REPEAT_ASK_GUEST",
            AskAndListen(
                tts_phrase="I am sorry, I did not understand. Please say 'Hi Tiago' for me to begin listening. What is your name and drink?",
            ),
            transitions={
                "succeeded": "PARSE_NAME",
                "failed": "failed",
            },
            remappings={"transcribed_speech": "guest_transcription"},
        )

    def check(self, blackboard):
        if self.flag:
            self.flag = False
            return "continue"
        else:
            return "succeeded"


def main():
    rclpy.init()

    yasmin_ros.set_ros_loggers()

    sm = yasmin.StateMachine(outcomes=["succeeded", "failed"], handle_sigint=True)

    sm.add_state(
        "NAME_DRINK",
        GetNameAndDrink(guest_id="guest1", last_resort=False),
        transitions={
            "succeeded": "succeeded",
            "failed": "failed",
            "failed_name": "failed",
            "failed_drink": "failed",
        },
    )

    bb = yasmin.Blackboard()
    bb["guest_transcription"] = "John"
    bb["guest_data"] = {
        "guest1": {
            "name": "",
            "drink": "",
            "detection": False,
            "seating_detection": False,
            "attributes": {},
            "seated_point": None,
        }
    }

    outcome = sm(bb)

    rclpy.shutdown()
