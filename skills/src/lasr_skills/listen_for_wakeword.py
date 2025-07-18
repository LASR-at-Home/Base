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


if __name__ == "__main__":
    import rospy

    rospy.init_node('test_wakeword')

    print("Testing Wakeword Detection")
    print("=" * 30)

    # Test 1: Only 'no'
    print("\nTest 1: Say 'no' (10 seconds)")
    sm1 = ListenForWakeword(wakeword="hi_tiago", timeout=10.0, threshold=0.3)
    outcome1 = sm1.execute()
    print(f"Result: {outcome1}")
    if 'keyword' in sm1.userdata:
        print(f"Detected: {sm1.userdata.keyword}")

    # Test 2: 'yes' or 'no'
    print("\nTest 2: Say 'yes' or 'no' (10 seconds)")
    sm2 = ListenForWakeword(wakeword=["yes", "no"], timeout=10.0, threshold=0.3)
    outcome2 = sm2.execute()
    print(f"Result: {outcome2}")
    if 'keyword' in sm2.userdata:
        print(f"Detected: {sm2.userdata.keyword}")

    print("\nTests completed!")

