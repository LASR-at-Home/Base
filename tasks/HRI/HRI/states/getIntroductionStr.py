import yasmin
from yasmin import Blackboard


class GetIntroductionStr(yasmin.State):

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("relevant_guest_data")
        self.add_input_key("introduce_to")
        self.add_output_key("text")

    def execute(self, blackboard: Blackboard) -> str:
        guest_to_introduce_data = blackboard["relevant_guest_data"]
        guest_to_introduce_to = blackboard["introduce_to"]

        blackboard["text"] = (
            f"Hello {guest_to_introduce_to}, "
            f"this is {guest_to_introduce_data['name']}. "
            f"Their favourite drink is {guest_to_introduce_data['drink']} "
        )
        return "succeeded"
