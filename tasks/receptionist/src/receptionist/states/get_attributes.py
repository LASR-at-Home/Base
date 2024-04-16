"""
State for calling the service to get a set of guest attributes.
Currently incomplete.
"""

import rospy
import smach
from smach import UserData
from typing import List, Any, Dict, Union


class GetGuestAttributes(smach.State):
    def __init__(
        self,
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

        self._attribute_service: Union[str, None] = attribute_service

    def _call_attribute_service(self):
        # TODO
        pass

    def execute(self, userdata: UserData) -> str:
        if self._attribute_service:
            attributes = self._call_attribute_service()
        else:
            attributes = {
                "hair_colour": "black",
                "glasses": True,
                "hat": False,
                "height": "short",
            }

        userdata.guest_data[userdata.guest_id]["attributes"] = attributes

        return "succeeded"