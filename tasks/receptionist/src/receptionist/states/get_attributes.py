import smach
from smach import UserData
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

    def __init__(
        self,
        guest_id: str,
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_id", "guest_data"],
            output_keys=["guest_data"],
        )
        self._guest_id: str = guest_id

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
