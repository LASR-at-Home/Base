"""
State for calling the service to get a set of guest attributes.
Currently incomplete.
"""

import rospy
import smach
from smach import UserData
from typing import List, Any, Dict, Union
from lasr_vision_msgs.srv import VqaRequest, VqaResponse, Vqa


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

    def _send_vqa_request(self, possible_answers: List[str]) -> str:
        request = VqaRequest()
        request.possible_answers = possible_answers
        response = self._service_proxy(request)
        return response.answer

    def execute(self, userdata: UserData) -> str:

        try: 
            if self._attribute_service:
                attributes = self._call_attribute_service()
            else:
                glasses_answers = [
                    "a person wearing glasses",
                    "a person not wearing glasses",
                ]
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

                glasses_response = self._send_vqa_request(glasses_answers)
                hat_response = self._send_vqa_request(hat_answers)
                height_response = self._send_vqa_request(height_answers)
                hair_colour_response = self._send_vqa_request(hair_colour_answers)

                print("glasses response: ", glasses_response)
                print("hat response", hat_response)
                print("height response", height_response)
                print("hair colour: ", hair_colour)

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

                attributes = {
                    "hair_colour": hair_colour,
                    "glasses": glasses,
                    "hat": hat,
                    "height": height,
                }

            userdata.guest_data[self._guest_id]["attributes"] = attributes
        except: 
            print("SOmething broke")
            attributes = {
                "hair_colour": "unknown",
                "glasses": "unknown",
                "hat": False,
                "height": "unknown",
            }
            userdata.guest_data[self._guest_id]["attributes"] = attributes




        return "succeeded"
