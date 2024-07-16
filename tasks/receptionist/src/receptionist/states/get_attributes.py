import smach
from smach import UserData
from lasr_skills import DescribePeople
import json


class GetGuestAttributes(smach.StateMachine):
    class InitialiseDetectionFlag(smach.State):
        def __init__(self, guest_id: str):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["guest_data", "clip_detection_dict"],
                output_keys=["guest_data"],
            )

            self._guest_id: str = guest_id

        def execute(self, userdata: UserData) -> str:
            try:
                userdata.guest_data[self._guest_id]["detection"] = False
                return "succeeded"
            except Exception as e:
                print(e)
                return "failed"

    class HandleGuestAttributes(smach.State):
        def __init__(self, guest_id: str):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["guest_data", "clip_detection_dict"],
                output_keys=["guest_data"],
            )

            self._guest_id: str = guest_id

        def execute(self, userdata: UserData) -> str:
            userdata.guest_data[self._guest_id][
                "attributes"
            ] = userdata.clip_detection_dict
            userdata.guest_data[self._guest_id]["detection"] = True
            return "succeeded"

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

        with self:
            smach.StateMachine.add(
                "INITIALISE_DETECTION_FLAG",
                self.InitialiseDetectionFlag(self._guest_id),
                transitions={
                    "succeeded": "GET_GUEST_ATTRIBUTES",
                    "failed": "GET_GUEST_ATTRIBUTES",
                },
            )
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
