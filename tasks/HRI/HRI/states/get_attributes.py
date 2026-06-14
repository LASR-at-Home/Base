import yasmin
from lasr_skills import DescribePeople
import json


class GetGuestAttributes(yasmin.StateMachine):
    class InitialiseDetectionFlag(yasmin.State):
        def __init__(self, guest_id: str):
            super().__init__(
                outcomes=["succeeded", "failed"],
            )

            self.add_input_key("guest_data")
            self.add_output_key("guest_data")

            self._guest_id: str = guest_id

        def execute(self, blackboard) -> str:
            try:
                blackboard["guest_data"][self._guest_id]["detection"] = False
                return "succeeded"
            except Exception as e:
                yasmin.YASMIN_LOG_ERROR(e)
                return "failed"

    class HandleGuestAttributes(yasmin.State):
        def __init__(self, guest_id: str):
            super().__init__(
                outcomes=["succeeded", "failed"],
            )

            self.add_input_key("guest_data")
            self.add_input_key("clip_detection_dict")
            self.add_output_key("guest_data")

            self._guest_id: str = guest_id

        def execute(self, blackboard) -> str:
            try:
                blackboard["guest_data"][self._guest_id]["attributes"] = blackboard[
                    "clip_detection_dict"
                ]
                blackboard["guest_data"][self._guest_id]["detection"] = True
                return "succeeded"
            except Exception as e:
                yasmin.YASMIN_LOG_ERROR(e)
                return "failed"

    def __init__(self, guest_id: str):
        super().__init__(
            outcomes=["succeeded", "failed"],
        )

        self.add_input_key("guest_data")
        self.add_output_key("guest_data")

        self._guest_id: str = guest_id

        self.add_state(
            "INITIALISE_DETECTION_FLAG",
            self.InitialiseDetectionFlag(guest_id=self._guest_id),
            transitions={
                "succeeded": "GET_GUEST_ATTRIBUTES",
                "failed": "failed",
            },
        )
        self.add_state(
            "GET_GUEST_ATTRIBUTES",
            DescribePeople(),
            transitions={
                "succeeded": "HANDLE_GUEST_ATTRIBUTES",
                "failed": "failed",
            },
        )
        self.add_state(
            "HANDLE_GUEST_ATTRIBUTES",
            self.HandleGuestAttributes(guest_id=self._guest_id),
            transitions={"succeeded": "succeeded", "failed": "failed"},
        )
