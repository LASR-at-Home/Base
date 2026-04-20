from typing import List, Union

import smach
import smach_ros

from lasr_speech_recognition_interfaces.srv import Wakeword


class ListenForWakeword(smach.StateMachine):

    def __init__(
        self, wakeword: Union[str, List[str]], timeout: float, threshold: float
    ) -> None:

        super(ListenForWakeword, self).__init__(
            outcomes=["succeeded", "failed"], output_keys=["keyword"]
        )

        # Normalise to list
        if isinstance(wakeword, str):
            wakeword = [wakeword]

        # Built a ROS2 Wakeword.Request object 
        request = Wakeword.Request()
        request.keywords = list(wakeword)
        request.timeout = float(timeout)
        request.threshold = float(threshold)

        with self:
            smach.StateMachine.add(
                "LISTEN_FOR_WAKEWORD",
                smach_ros.ServiceState(
                    "/lasr_wakewords/detect",
                    Wakeword,
                    request=request,
                    response_slots=["success", "keyword"],
                ),
                transitions={
                    "succeeded": "DETERMINE_OUTCOME",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "DETERMINE_OUTCOME",
                smach.CBState(
                    lambda ud: "succeeded" if ud.success else "failed",
                    outcomes=["succeeded", "failed"],
                    input_keys=["success"],
                ),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
