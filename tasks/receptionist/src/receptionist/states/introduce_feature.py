from typing import Any, Dict, List, Optional

import rospy
import smach
from lasr_skills import Say
from smach import UserData

def stringify_guest_data(
        guest_data: Dict[str, Any], guest_id: str,
    ) -> str:
        """Converts the guest data for a specified guest into a string that can be used
        for the robot to introduce the guest to the other guests/host.

        Args:
            guest_data (Dict[str, Any]): guest data dictionary.
            guest_id (str): guest id key to use to get the guest data.

        Returns:
            str: string representation of the guest data.
        """

        relevant_guest_data = guest_data[guest_id]

        guest_str = f"I will describe how {relevant_guest_data['name']} looks. "

        if (
            not relevant_guest_data["detection"]
            or "attributes" not in relevant_guest_data
        ):
            return guest_str

        relevant_guest_data["attributes"]["has_hair"] = 0.5

        if relevant_guest_data["attributes"]["long_hair"]:
            guest_str += "They have long hair. "
        else:
            guest_str += "They have short hair. "

        t_shirt = (
            "short sleeve"
            if relevant_guest_data["attributes"]["short_sleeve_t_shirt"]
            else "long sleeve"
        )

        if (
            relevant_guest_data["attributes"]["glasses"]
            and relevant_guest_data["attributes"]["hat"]
        ):
            guest_str += f"They are wearing a {t_shirt} top, glasses and a hat. "
        elif (
            relevant_guest_data["attributes"]["glasses"]
            and not relevant_guest_data["attributes"]["hat"]
        ):
            guest_str += f"They are wearing a {t_shirt} t shirt and glasses and they are not wearing a hat. "
        elif (
            not relevant_guest_data["attributes"]["glasses"]
            and relevant_guest_data["attributes"]["hat"]
        ):
            guest_str += f"They wearing a {t_shirt} t shirt and hat and they are not wearing glasses. "
        elif (
            not relevant_guest_data["attributes"]["glasses"]
            and not relevant_guest_data["attributes"]["hat"]
        ):
            guest_str += f"They wearing a {t_shirt} t shirt and they are not wearing glasses or a hat. "
        return guest_str

class GetStrGuestData(smach.State):
    def __init__(self, guest_id: str):
        super().__init__(
            outcomes=["succeeded"],
            input_keys=["guest_data"],
            output_keys=["guest_str"],
        )
        self._guest_id = guest_id

    def execute(self, userdata: UserData) -> str:
        guest_str = stringify_guest_data(
            userdata.guest_data, self._guest_id,
        )
        userdata.guest_str = guest_str
        return "succeeded"
    
class IntroduceFeature(smach.StateMachine):
    def __init__(self, guest_id):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data"],
        )

        with self:
            smach.StateMachine.add(
                "GetIntroductionString",
                GetStrGuestData(
                    guest_id=guest_id,
                ),
                transitions={"succeeded": "SayIntroduceFeature"},
            )

            smach.StateMachine.add(
                "SayIntroduceFeature",
                Say(),
                transitions={
                    "succeeded": "succeeded",
                    "preempted": "failed",
                    "aborted": "failed",
                },
                remapping={"text": "guest_str"},
            )