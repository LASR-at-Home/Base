import smach
from smach import UserData
<<<<<<< HEAD
from typing import List, Any, Dict, Union
from lasr_vision_msgs.srv import VqaRequest, VqaResponse, Vqa

=======
from lasr_skills import DescribePeople
import json


class GetGuestAttributes(smach.StateMachine):
    class HandleGuestAttributes(smach.State):
        def __init__(self, guest_id: str):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["people", "guest_data"],
                output_keys=["guest_data"],
            )

            self._guest_id: str = guest_id

        def execute(self, userdata: UserData) -> str:
            if len(userdata.people) == 0:
                return "failed"
            userdata.guest_data[self._guest_id]["attributes"] = json.loads(
                userdata.people[0]["features"]
            )["attributes"]
            return "succeeded"
>>>>>>> 5026ebfb0cc02564e84da9d05b79c6aa6d85b8f3

    def __init__(
        self,
        guest_id: str,
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data"],
            output_keys=["guest_data"],
        )
        self._guest_id: str = guest_id
<<<<<<< HEAD
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
                "hat": "no hat",
                "height": "unknown",
            }
            userdata.guest_data[self._guest_id]["attributes"] = attributes



=======
>>>>>>> 5026ebfb0cc02564e84da9d05b79c6aa6d85b8f3

        with self:
            smach.StateMachine.add(
                "GET_GUEST_ATTRIBUTES",
                DescribePeople(),
                transitions={
                    "succeeded": "HANDLE_GUEST_ATTRIBUTES",
                    "failed": "failed",
                },
            )
            smach.StateMachine.add(
                "HANDLE_GUEST_ATTRIBUTES",
                self.HandleGuestAttributes(self._guest_id),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )
