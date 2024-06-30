"""
State machine that introduces the greeted guest to all other guests/host present in the 
seating area.

"""

import rospy
import smach
from smach import UserData
from lasr_skills import Say
from typing import Dict, List, Any, Optional


def stringify_guest_data(guest_data: Dict[str, Any], guest_id: str) -> str:
    """Converts the guest data for a specified guest into a string that can be used
    for the robot to introduce the guest to the other guests/host.

    Args:
        guest_data (Dict[str, Any]): guest data dictionary.
        guest_id (str): guest id key to use to get the guest data.

    Returns:
        str: string representation of the guest data.
    """

    relevant_guest_data = guest_data[guest_id]
    relevant_guest_data.setdefault(
        "attributes",
        {
            "has_hair": 0,
            "hair_shape": "unknown",
            "hair_colour": "unknown",
            "facial_hair": 0,
            "earrings": 0,
            "necklace": 0,
            "necktie": 0,
            # "height": "unknown",
            "glasses": 0,
            "hat": 0,
            "dress": 0,
            "top": 0,
            "outwear": 0,
            "max_dress": "unknown",
            "max_top": "unknown",
            "max_outwear": "unknown",
        },
    )

    guest_str = ""

    guest_str += f"{relevant_guest_data['name']}, their favourite drink is {relevant_guest_data['drink']}. "

    if not relevant_guest_data["detection"]:
        return guest_str

    filtered_attributes = {}
    filtered_attributes["hair"] = {
        "confidence": relevant_guest_data["attributes"]["has_hair"],
        "hair_shape": relevant_guest_data["attributes"]["hair_shape"],
        "hair_colour": relevant_guest_data["attributes"]["hair_colour"],
    }

    most_confident_clothes = find_most_confident_clothes(
        relevant_guest_data,
        clothes=[
            ["dress", relevant_guest_data["attributes"]["dress"]],
            ["top", relevant_guest_data["attributes"]["top"]],
            ["outwear", relevant_guest_data["attributes"]["outwear"]],
        ],
    )
    filtered_attributes["clothes"] = {
        "confidence": most_confident_clothes[0],
        "attribute": most_confident_clothes[1],
    }

    considered_attributes = [
        "dress",
        "top",
        "outwear",
        "max_dress",
        "max_top",
        "max_outwear",
        "has_hair",
        "hair_shape",
        "hair_colour",
    ]
    for attribute, value in relevant_guest_data["attributes"].items():
        if attribute not in considered_attributes:
            filtered_attributes[attribute] = {
                "confidence": relevant_guest_data["attributes"][attribute],
                "attribute": attribute,
            }

    sorted_attributes = sorted(
        filtered_attributes.keys(),
        key=lambda k: abs(filtered_attributes[k]["confidence"]),
        reverse=True,
    )

    top_4_attributes = sorted_attributes[:4]

    top_4_attributes = [
        attr
        for attr in top_4_attributes
        if not (attr == "hair" and filtered_attributes[attr]["confidence"] < 0)
    ]
    if len(top_4_attributes) < 4:
        top_4_attributes.append(sorted_attributes[4])

    for i in range(len(top_4_attributes)):
        attribute_name = top_4_attributes[i]
        attribute_value = filtered_attributes[top_4_attributes[i]]
        confidence = attribute_value["confidence"]

        if attribute_name == "hair":
            hair_shape = attribute_value["hair_shape"]
            hair_colour = attribute_value["hair_colour"]
            guest_str += f"They have {hair_shape} and {hair_colour} . "
        elif attribute_name == "facial_hair":
            if confidence < 0:
                guest_str += "They don't have facial hair. "
            else:
                guest_str += "They have facial hair. "
        else:
            if confidence < 0:
                if isSingular(attribute_value["attribute"]):
                    guest_str += (
                        f"They are not wearing a {attribute_value['attribute']}."
                    )
                else:
                    guest_str += f"They are not wearing {attribute_value['attribute']}."
            else:
                if isSingular(attribute_value["attribute"]):
                    guest_str += f"They are wearing a {attribute_value['attribute']}."
                else:
                    guest_str += f"They are wearing {attribute_value['attribute']}."

    return guest_str


def find_most_confident_clothes(
    relevant_guest_data: Dict[str, Any], clothes: List[List[Any]]
) -> List[Any]:
    """Find the clothes it's most confident of, after determining the clothes type
    through confidence values.

    Args:
        relevant_guest_data (Dict[str, Any]): guest data dictionary.
        clothes List[List[Any]]: List of the clothes type and their confidence

    Returns:
        List: Maximum confidence and the relevant clothes
    """
    max_clothes_type, max_confidence = max(clothes, key=lambda c: c[1])

    if max_clothes_type == "dress":
        max_clothes = relevant_guest_data["attributes"]["max_dress"]
    elif max_clothes_type == "top":
        max_clothes = relevant_guest_data["attributes"]["max_top"]
    else:
        max_clothes = relevant_guest_data["attributes"]["max_outwear"]

    return [max_confidence, max_clothes]


def isSingular(attribute: str):
    """Checks if a word is singular or plural by checking the last letter

    Args:
        attribute (str): The attribute to check for plurality

    Returns:
        (bool): Boolean identifying whether the word is plural
    """
    if attribute[len(attribute) - 1] == "s":
        return False
    else:
        return True


class GetStrGuestData(smach.State):
    def __init__(self, guest_id: str):
        super().__init__(
            outcomes=["succeeded"],
            input_keys=["guest_data"],
            output_keys=["guest_str"],
        )
        self._guest_id = guest_id

    def execute(self, userdata: UserData) -> str:
        guest_str = stringify_guest_data(userdata.guest_data, self._guest_id)
        userdata.guest_str = guest_str
        return "succeeded"


class GetGuestName(smach.State):
    def __init__(self, guest_id: str):
        super().__init__(
            outcomes=["succeeded"],
            input_keys=["guest_data"],
            output_keys=["requested_name"],
        )
        self._guest_id = guest_id

    def execute(self, userdata: UserData) -> str:
        requested_name = userdata.guest_data[self._guest_id]["name"]
        userdata.requested_name = requested_name
        return "succeeded"


class GetIntroductionString(smach.State):
    def __init__(self):
        super().__init__(
            outcomes=["succeeded"],
            input_keys=["guest_str", "requested_name"],
            output_keys=["introduction_str"],
        )

    def execute(self, userdata: UserData) -> str:
        introduction_str = (
            f"Hello {userdata.requested_name}, this is {userdata.guest_str}."
        )
        userdata.introduction_str = introduction_str
        return "succeeded"


class Introduce(smach.StateMachine):
    def __init__(
        self,
        guest_to_introduce: str,
        guest_to_introduce_to: Optional[str] = None,
        everyone: Optional[bool] = False,
    ):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data"],
        )
        assert not (guest_to_introduce_to is None and not everyone)

        with self:
            if everyone:
                smach.StateMachine.add(
                    "GetStrGuestData",
                    GetStrGuestData(guest_id=guest_to_introduce),
                    transitions={"succeeded": "SayIntroduce"},
                )
                smach.StateMachine.add(
                    "SayIntroduce",
                    Say(
                        format_str="Hello everyone, this is {}.",
                    ),
                    transitions={
                        "succeeded": "succeeded",
                        "preempted": "failed",
                        "aborted": "failed",
                    },
                    remapping={"placeholders": "guest_str"},
                )

            else:
                smach.StateMachine.add(
                    "GetStrGuestData",
                    GetStrGuestData(guest_id=guest_to_introduce),
                    transitions={"succeeded": "GetGuestName"},
                )

                smach.StateMachine.add(
                    "GetGuestName",
                    GetGuestName(guest_id=guest_to_introduce_to),
                    transitions={"succeeded": "GetIntroductionString"},
                )

                smach.StateMachine.add(
                    "GetIntroductionString",
                    GetIntroductionString(),
                    transitions={"succeeded": "SayIntroduce"},
                    remapping={
                        "guest_str": "guest_str",
                        "requested_name": "requested_name",
                    },
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
