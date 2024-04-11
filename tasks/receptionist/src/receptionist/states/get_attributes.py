"""
State for calling the service to get a set of guest attributes.
Currently incomplete.
"""

import rospy
import smach
from typing import List, Any, Dict


class GetGuestAttributes(smach.State):
    def __init__(
        self,
        attribute_service: str,
        outcomes: List[str] = ["succeeded", "failed"],
        input_keys: List[str] = ["guest id", "guest data"],
        output_keys: List[str] = ["guest data"],
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

        self._attribute_service: str = attribute_service

    def _call_attribute_service(self):
        # TODO
        pass

    def execute(self, userdata: Dict[str, Any]) -> str:
        outcome = "succeeded"
        return outcome
