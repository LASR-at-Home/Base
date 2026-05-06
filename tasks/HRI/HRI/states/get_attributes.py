import smach
import smach_ros
from smach import UserData
from skills.src.lasr_skills import DescribePeople
import json


class GetGuestAttributes(smach.StateMachine):
    class InitialiseDetectionFlag(smach_ros.RosState):
        def __init__(self, guest_id: str, node):
            super().__init__(
                node=node,
                outcomes=["succeeded", "failed"],
                input_keys=["guest_data"],
                output_keys=["guest_data"],
            )

            self._guest_id: str = guest_id

        def execute(self, userdata: UserData) -> str:
            try:
                userdata.guest_data[self._guest_id]["detection"] = False
                return "succeeded"
            except Exception as e:
                self.node.get_logger().error(f"Error: {e}")
                return "failed"

    class HandleGuestAttributes(smach_ros.RosState):
        def __init__(self, guest_id: str, node):
            super().__init__(
                node=node,
                outcomes=["succeeded", "failed"],
                input_keys=["guest_data", "clip_detection_dict"],
                output_keys=["guest_data"],
            )

            self._guest_id: str = guest_id

        def execute(self, userdata: UserData) -> str:
            try:
                userdata.guest_data[self._guest_id][
                    "attributes"
                ] = userdata.clip_detection_dict
                userdata.guest_data[self._guest_id]["detection"] = True
                return "succeeded"
            except Exception as e:
                self.node.get_logger().error(f"Error: {e}")
                return 'failed'

    def __init__(self, guest_id: str, node):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data"],
            output_keys=["guest_data"],
        )
        self._guest_id: str = guest_id

        with self:
            self.add(
                "INITIALISE_DETECTION_FLAG",
                self.InitialiseDetectionFlag(self._guest_id),
                transitions={
                    "succeeded": "GET_GUEST_ATTRIBUTES",
                    "failed": "GET_GUEST_ATTRIBUTES",
                },
            )
            self.add(
                "GET_GUEST_ATTRIBUTES",
                DescribePeople(),
                transitions={
                    "succeeded": "HANDLE_GUEST_ATTRIBUTES",
                    "failed": "failed",
                },
            )
            self.add(
                "HANDLE_GUEST_ATTRIBUTES",
                self.HandleGuestAttributes(self._guest_id),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )