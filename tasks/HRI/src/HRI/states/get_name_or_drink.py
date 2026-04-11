"""
State for parsing the transcription of the guests' information (favourite drink or name), and adding this
to the guest data userdata
"""

import rclpy
from rclpy.node import Node

import smach
from smach import UserData
from typing import List, Dict, Any
from HRI.states import SpeechRecovery

import smach
from smach import UserData
from typing import List, Dict, Any, Optional
from .speech_recovery import SpeechRecovery


class GetNameOrDrink(smach.StateMachine):
    class ParseTranscribedInfo(RosState):
        def __init__(
            self, node: Node, guest_id: str, info_type: str, param_key: str = "priors"
        ):
            """Parses the transcription of the guests' information.

            Args:
                guest_id (str): ID of the guest (identifying the guest)
                info_type (str): The type of information to try and extract useful information
                (drink or name)

                # Paramters are node specific in ros2
                param_key (str, optional): Name of the parameter that contains the list of
                possible . Defaults to "receptionist/priors".
            """
            RosState.__init__(
                self,
                node=node,
                outcomes=["succeeded", "failed"],
                input_keys=["guest_transcription", "guest_data"],
                output_keys=["guest_data", "guest_transcription"],
            )
            self._guest_id = guest_id
            self._type = info_type

            possible_names = [
                name.lower()
                for name in self.node.get_parameter(f"{param_key}.names").value
            ]
            possible_drinks = [
                drink.lower()
                for drink in self.node.get_parameter(f"{param_key}.drinks").value
            ]

            self._possible_information = {
                "drink": possible_drinks,
                "name": possible_names,
            }[self._type]

        def execute(self, userdata: UserData) -> str:
            """Parse the guest's information.

            Args:
                userdata (UserData): State machine userdata assumed to contain a key
                called "guest transcription" with the transcription of the guest's name or
                favourite drink or both.

            Returns:
                str: state outcome. Updates the userdata with the parsed information (drink or name), under
                the parameter "guest data".
            """
            outcome = "succeeded"
            information_found = False
            transcription = userdata.guest_transcription.lower()

            transcription = userdata["guest_transcription"].lower()

            for key_phrase in self._possible_information:
                if key_phrase in transcription:
                    userdata.guest_data[self._guest_id][self._type] = key_phrase
                    self.node.get_logger().info(
                        f"Guest/Drink {self._type} identified as: {key_phrase}"
                    )
                    information_found = True
                    break
            if not information_found:
                self.node.get_logger().info(f"{self._type} not found in transcription")
                userdata.guest_data[self._guest_id][self._type] = "unknown"
                outcome = "failed"

            return outcome

    def __init__(
        self,
        node: Node,
        guest_id: str,
        last_resort: bool,
        info_type: str,
        param_key: str = "priors",
    ):
        self._last_resort = last_resort
        self._guest_id = guest_id
        self._info_type = info_type
        self._param_key = param_key

        self.__node = node

        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_transcription", "guest_data"],
            output_keys=["guest_data", "guest_transcription"],
        )
        with self:

            smach.StateMachine.add(
                "PARSE_NAME_OR_DRINK",
                self.ParseTranscribedInfo(
                    node=self.__node,
                    guest_id=self._guest_id,
                    info_type=self._info_type,
                    param_key=self._param_key,
                ),
                transitions={"succeeded": "succeeded", "failed": "SPEECH_RECOVERY"},
            )
            smach.StateMachine.add(
                "SPEECH_RECOVERY",
                SpeechRecovery(self._guest_id, self._last_resort, self._info_type),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
