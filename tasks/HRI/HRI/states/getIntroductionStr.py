import smach
from smach import UserData


class GetIntroductionStr(smach.State):
    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["relevant_guest_data", "introduce_to"],
            output_keys=["introduction_str"],
        )

    def execute(self, userdata: UserData) -> str:

        guest_to_introduce_data = userdata.relevant_guest_data
        guest_to_introduce_to = userdata.introduce_to

        introduction_str = (
            f"Hello {guest_to_introduce_to}, "
            f"this is {guest_to_introduce_data['name']}. "
            f"Their favourite drink is {guest_to_introduce_data['drink']}, "
            f"and their interest is {guest_to_introduce_data['interest']}."
        )
        userdata.introduction_str = introduction_str
        return "succeeded"
