import smach_ros
import smach

from lasr_speech_recognition_msgs.srv import WakewordTrigger, WakewordTriggerRequest


class ListenForWakeword(smach.StateMachine):

    def __init__(self, wakeword: str, timeout: float, threshold: float) -> None:

        super(ListenForWakeword, self).__init__(outcomes=["succeeded", "failed"])

        with self:
            smach.StateMachine.add(
                "LISTEN_FOR_WAKEWORD",
                smach_ros.ServiceState(
                    "/lasr_wakewords/detect",
                    WakewordTrigger,
                    request=WakewordTriggerRequest(wakeword, timeout, threshold),
                    response_slots=["success"],
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
