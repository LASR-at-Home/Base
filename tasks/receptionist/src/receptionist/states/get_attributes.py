"""
State for calling the service to get a set of guest attributes.
Currently incomplete.
"""

import rospy
import smach
from smach import UserData
from typing import List, Any, Dict, Union
from lasr_vision_clip.srv import VqaRequest, VqaResponse, Vqa


class GetGuestAttributes(smach.State):
    def __init__(
        self,
        guest_id: str,
        attribute_service: Union[str, None] = None,
        outcomes: List[str] = ["succeeded", "failed"],
        input_keys: List[str] = ["guest_id", "guest_data"],
        output_keys: List[str] = ["guest_data"],
    ):
        """Calls and parses the service that gets a set of guest attributes.

        Args:
            attribute_service (str): Name of the service to call that returns the guest's attributes.
        """

        super().__init__(
            outcomes=outcomes,
            input_keys=input_keys,
            output_keys=output_keys,
        )
        self._service_proxy = rospy.ServiceProxy("/clip_vqa/query_service", Vqa)
        self._guest_id: str = guest_id
        self._attribute_service: Union[str, None] = attribute_service

    def _call_attribute_service(self):
        # TODO
        pass

    def execute(self, userdata: UserData) -> str:
        if self._attribute_service:
            attributes = self._call_attribute_service()
        glasses_answers = ["a person wearing glasses", "a person not wearing glasses"]
        hat_answers = ["a person wearing a hat", "a person not wearing a hat"]
        height_answers = ["a tall person", "a short person"]
        hair_colour_answers = [
            "a person with black hair",
            "a person with blonde hair",
            "a person with brown hair",
            "a person with red hair",
            "a person with grey hair",
            "a person with white hair",
        ]
        glasses_request = VqaRequest()
        glasses_request.possible_answers = glasses_answers
        glasses_response = self._service_proxy(glasses_request)
        glasses_response = glasses_response.answer
        hat_request = VqaRequest()
        hat_request.possible_answers = hat_answers
        hat_response = self._service_proxy(hat_request)
        hat_response = hat_response.answer
        height_request = VqaRequest()
        height_request.possible_answers = height_answers
        height_response = self._service_proxy(height_request)
        height_response = height_response.answer
        hair_colour_request = VqaRequest()
        hair_colour_request.possible_answers = hair_colour_answers
        hair_colour_response = self._service_proxy(hair_colour_request)
        hair_colour_response = hair_colour_response.answer
        if glasses_response == "a person wearing glasses":
            glasses = True
        else:
            glasses = False
        if hat_response == "a person wearing a hat":
            hat = True
        else:
            hat = False
        if height_response == "a tall person":
            height = "tall"
        else:
            height = "short"
        if hair_colour_response == "a person with black hair":
            hair_colour = "black"
        elif hair_colour_response == "a person with blonde hair":
            hair_colour = "blonde"
        elif hair_colour_response == "a person with brown hair":
            hair_colour = "brown"
        elif hair_colour_response == "a person with red hair":
            hair_colour = "red"
        elif hair_colour_response == "a person with grey hair":
            hair_colour = "grey"

        else:
            attributes = {
                "hair_colour": hair_colour,
                "glasses": glasses,
                "hat": hat,
                "height": height,
            }

        userdata.guest_data[self._guest_id]["attributes"] = attributes

        return "succeeded"
