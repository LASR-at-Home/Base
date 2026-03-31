import smach
from smach import UserData, StateMachine
from smach_ros import RosState

from skills import DescribePeople
import json


class GetGuestAttributes(StateMachine):
    class InitialiseDetectionFlag(RosState):
        def __init__(self, node, guest_id: str):
            RosState.__init__(
                self,
                node,
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

    class HandleGuestAttributes(RosState):
        def __init__(self, node, guest_id: str):
            RosState.__init__(
                self,
                node,
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

    def __init__(self, node, guest_id: str):
        StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data"],
            output_keys=["guest_data"],
        )
        self._guest_id: str = guest_id
        self.__node = node

        with self:
            StateMachine.add(
                "INITIALISE_DETECTION_FLAG",
                self.InitialiseDetectionFlag(self.__node, self._guest_id),
                transitions={
                    "succeeded": "GET_GUEST_ATTRIBUTES",
                    "failed": "GET_GUEST_ATTRIBUTES",
                },
            )
            StateMachine.add(
                "GET_GUEST_ATTRIBUTES",
                DescribePeople(self.__node),
                transitions={
                    "succeeded": "HANDLE_GUEST_ATTRIBUTES",
                    "failed": "failed",
                },
            )
            StateMachine.add(
                "HANDLE_GUEST_ATTRIBUTES",
                self.HandleGuestAttributes(self.__node, self._guest_id),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
