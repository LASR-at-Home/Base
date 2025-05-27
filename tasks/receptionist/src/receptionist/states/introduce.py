"""
State machine that introduces the greeted guest to all other guests/host present in the
seating area.

"""

from typing import Any, Dict, List, Optional

import rospy
import smach
from lasr_skills import LookToPoint, Say
from smach import UserData

# import llm_utils


# def stringify_guest_data(
#     guest_data: Dict[str, Any], guest_id: str, describe_features: bool
# ) -> str:
#     """Converts the guest data for a specified guest into a string that can be used
#     for the robot to introduce the guest to the other guests/host.

#     Args:
#         guest_data (Dict[str, Any]): guest data dictionary.
#         guest_id (str): guest id key to use to get the guest data.

#     Returns:
#         str: string representation of the guest data.
#     """

#     relevant_guest_data = guest_data[guest_id]

#     guest_str = f"{relevant_guest_data['name']}, their favourite drink is {relevant_guest_data['drink']} and interest is {relevant_guest_data['interest']}. "

#     if (
#         not relevant_guest_data["detection"]
#         or not describe_features
#         or "attributes" not in relevant_guest_data
#     ):
#         return guest_str

#     relevant_guest_data["attributes"]["has_hair"] = 0.5

#     if relevant_guest_data["attributes"]["long_hair"]:
#         guest_str += "They have long hair. "
#     else:
#         guest_str += "They have short hair. "

#     t_shirt = (
#         "short sleeve"
#         if relevant_guest_data["attributes"]["short_sleeve_t_shirt"]
#         else "long sleeve"
#     )

#     if (
#         relevant_guest_data["attributes"]["glasses"]
#         and relevant_guest_data["attributes"]["hat"]
#     ):
#         guest_str += f"They are wearing a {t_shirt} top, glasses and a hat. "
#     elif (
#         relevant_guest_data["attributes"]["glasses"]
#         and not relevant_guest_data["attributes"]["hat"]
#     ):
#         guest_str += f"They are wearing a {t_shirt} t shirt and glasses and they are not wearing a hat. "
#     elif (
#         not relevant_guest_data["attributes"]["glasses"]
#         and relevant_guest_data["attributes"]["hat"]
#     ):
#         guest_str += f"They wearing a {t_shirt} t shirt and hat and they are not wearing glasses. "
#     elif (
#         not relevant_guest_data["attributes"]["glasses"]
#         and not relevant_guest_data["attributes"]["hat"]
#     ):
#         guest_str += f"They wearing a {t_shirt} t shirt and they are not wearing glasses or a hat. "
#     return guest_str


# class GetStrGuestData(smach.State):
#     def __init__(self, guest_id: str, describe_features: bool = False):
#         super().__init__(
#             outcomes=["succeeded"],
#             input_keys=["guest_data"],
#             output_keys=["guest_str"],
#         )
#         self._guest_id = guest_id
#         self._describe_features = describe_features

#     def execute(self, userdata: UserData) -> str:
#         guest_str = stringify_guest_data(
#             userdata.guest_data, self._guest_id, self._describe_features
#         )
#         userdata.guest_str = guest_str
#         return "succeeded"


# class GetGuestName(smach.State):
#     def __init__(self, guest_id: str):
#         super().__init__(
#             outcomes=["succeeded"],
#             input_keys=["guest_data"],
#             output_keys=["requested_name"],
#         )
#         self._guest_id = guest_id

#     def execute(self, userdata: UserData) -> str:
#         requested_name = userdata.guest_data[self._guest_id]["name"]
#         userdata.requested_name = requested_name
#         return "succeeded"


# class GetIntroductionString(smach.State):
#     def __init__(self):
#         super().__init__(
#             outcomes=["succeeded"],
#             input_keys=["guest_str", "requested_name"],
#             output_keys=["introduction_str"],
#         )

#     def execute(self, userdata: UserData) -> str:
#         introduction_str = (
#             f"Hello {userdata.requested_name}, this is {userdata.guest_str}."
#         )
#         userdata.introduction_str = introduction_str
#         return "succeeded"
import smach
from smach import UserData
import llm_utils

class GetIntroductionString(smach.State):
    def __init__(self, guest_to_introduce: str, guest_to_introduce_to: str = None):
        super().__init__(
            outcomes=["succeeded"],
            input_keys=["guest_data", "host_data"],
            output_keys=["introduction_str"],
        )
        self.guest_to_introduce = guest_to_introduce
        self.guest_to_introduce_to = guest_to_introduce_to

    def _get_profile(self, guest_id: str, guest_data: dict, host_data: dict) -> dict:
        return host_data if guest_id == "host" else guest_data.get(guest_id, {})

    def execute(self, userdata: UserData) -> str:
        intro = ""

        introducer = self._get_profile(self.guest_to_introduce, userdata.guest_data, userdata.host_data)
        introducer_name = introducer.get("name", "Unknown")
        introducer_drink = introducer.get("drink", "Unknown")
        introducer_interest = introducer.get("interest", "Unknown")

        # Case: Introduce to everyone (broadcast)
        if self.guest_to_introduce_to is None:
            llm_intro = llm_utils.introduce_llm(introducer_name, introducer_drink, introducer_interest)
            intro = f"Hello everyone, {llm_intro}"

        else:
            target = self._get_profile(self.guest_to_introduce_to, userdata.guest_data, userdata.host_data)
            target_name = target.get("name", "Unknown")

            llm_intro = llm_utils.introduce_llm(introducer_name, introducer_drink, introducer_interest)
            intro = f"Hello {target_name}, {llm_intro}"

            should_compare = False
            if self.guest_to_introduce == "host" and self.guest_to_introduce_to != "host":
                should_compare = True
            elif self.guest_to_introduce != "host" and self.guest_to_introduce_to != "host":
                should_compare = True

            if should_compare:
                comparison = llm_utils.interest_commonality_llm([
                    introducer_interest,
                    target.get("interest", "Unknown")
                ])
                intro += f" {comparison}"

        userdata.introduction_str = intro.strip()
        return "succeeded"

class Introduce(smach.StateMachine):
    def __init__(self, guest_to_introduce: str, guest_to_introduce_to: str = None):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data", "host_data"],
        )

        with self:
            smach.StateMachine.add(
                "GetIntroductionString",
                GetIntroductionString(
                    guest_to_introduce=guest_to_introduce,
                    guest_to_introduce_to=guest_to_introduce_to,
                ),
                transitions={"succeeded": "SayIntroduce"},
            )

            smach.StateMachine.add(
                "SayIntroduce",
                Say(),
                transitions={
                    "succeeded": "succeeded",
                    "preempted": "failed",
                    "aborted": "failed",
                },
                remapping={"text": "introduction_str"},
            )




# class Introduce(smach.StateMachine):
#     def __init__(
#         self,
#         guest_to_introduce: str,
#         guest_to_introduce_to: Optional[str] = None,
#         # describe_features: bool = False,
#         everyone: Optional[bool] = False,
#     ):
#         super().__init__(
#             outcomes=["succeeded", "failed"],
#             input_keys=["guest_data"],
#         )
#         assert not (guest_to_introduce_to is None and not everyone)

#         with self:
#             if everyone:
#                 smach.StateMachine.add(
#                     "GetStrGuestData",
#                     GetStrGuestData(
#                         guest_id=guest_to_introduce, describe_features=describe_features
#                     ),
#                     transitions={"succeeded": "SayIntroduce"},
#                 )
#                 smach.StateMachine.add(
#                     "SayIntroduce",
#                     Say(
#                         format_str="Hello everyone, this is {}.",
#                     ),
#                     transitions={
#                         "succeeded": "succeeded",
#                         "preempted": "failed",
#                         "aborted": "failed",
#                     },
#                     remapping={"placeholders": "guest_str"},
#                 )

#             else:
#                 smach.StateMachine.add(
#                     "GetStrGuestData",
#                     GetStrGuestData(
#                         guest_id=guest_to_introduce, describe_features=describe_features
#                     ),
#                     transitions={"succeeded": "GetGuestName"},
#                 )

#                 smach.StateMachine.add(
#                     "GetGuestName",
#                     GetGuestName(guest_id=guest_to_introduce_to),
#                     transitions={"succeeded": "GetIntroductionString"},
#                 )

#                 smach.StateMachine.add(
#                     "GetIntroductionString",
#                     GetIntroductionString(),
#                     transitions={"succeeded": "SayIntroduce"},
#                     remapping={
#                         "guest_str": "guest_str",
#                         "requested_name": "requested_name",
#                     },
#                 )

#                 smach.StateMachine.add(
#                     "SayIntroduce",
#                     Say(),
#                     transitions={
#                         "succeeded": "succeeded",
#                         "preempted": "failed",
#                         "aborted": "failed",
#                     },
#                     remapping={"text": "introduction_str"},
#                 )
