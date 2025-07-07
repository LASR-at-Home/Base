from typing import List, Union

import smach_ros
import smach

from lasr_speech_recognition_msgs.srv import Wakeword, WakewordRequest


class ListenForWakeword(smach.StateMachine):

    def __init__(
        self, wakeword: Union[str, List[str]], timeout: float, threshold: float
    ) -> None:

        super(ListenForWakeword, self).__init__(
            outcomes=["succeeded", "failed"], output_keys=["keyword"]
        )

        if isinstance(wakeword, str):
            wakeword = [wakeword]

        with self:
            smach.StateMachine.add(
                "LISTEN_FOR_WAKEWORD",
                smach_ros.ServiceState(
                    "/lasr_wakewords/detect",
                    Wakeword,
                    request=WakewordRequest(wakeword, timeout, threshold),
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
