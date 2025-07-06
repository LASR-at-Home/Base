import smach
import smach_ros
from geometry_msgs.msg import Pose, PointStamped
from std_msgs.msg import Header
from lasr_skills import (
    AskAndListen,
    GoToLocation,
    Rotate,
    Say,
    Wait,
    LookToPoint,
    Listen,
    ListenForWakeword,
)
from restaurant.states import (
    Survey,
    HandleOrder,
    GetPoses,
    ComputeApproach,
    SwitchToMappingMode,
)
from std_msgs.msg import Empty


class GetLookPoint(smach.State):
    """
    State to get the look point from the waving person detection.
    It sets the point in the userdata.
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["waving_person_detection"],
            output_keys=["waving_person_detection_point"],
        )

    def execute(self, userdata):
        if userdata.waving_person_detection:
            userdata.waving_person_detection_point = PointStamped(
                header=Header(frame_id="map"),
                point=userdata.waving_person_detection.point,
            )
            return "succeeded"
        else:
            return "failed"


class ChooseWavingPerson(smach.State):
    """
    State to choose a waving person from the list of detected people.
    The last person in the list is chosen
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["hands_up_detections"],
            output_keys=["waving_person_detection", "hands_up_detections"],
        )

    def execute(self, userdata):
        if userdata.hands_up_detections:
            userdata.waving_person_detection = userdata.hands_up_detections.pop()
            return "succeeded"
        else:
            return "failed"


class Restaurant(smach.StateMachine):

    def __init__(self) -> None:
        super().__init__(outcomes=["succeeded", "failed"])

        with self:
            smach.StateMachine.add(
                "WAIT_START",
                smach_ros.MonitorState(
                    "/restaurant/start",
                    Empty,
                    lambda _ud, _msg: False,
                ),
                transitions={
                    "valid": "WAIT_START",
                    "preempted": "WAIT_START",
                    "invalid": "SWITCH_TO_MAPPING_MODE",
                },
            )

            # In practie we'll do this in the queue
            smach.StateMachine.add(
                "SWITCH_TO_MAPPING_MODE",
                SwitchToMappingMode(),
                transitions={"succeeded": "SAY_START", "failed": "failed"},
            )

            smach.StateMachine.add(
                "SAY_START",
                Say(text="Start of Restaurant task."),
                transitions={
                    "succeeded": "ROTATE_360",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            # To attempt to build a map
            smach.StateMachine.add(
                "ROTATE_360",
                Rotate(angle=360.0),
                transitions={"succeeded": "GET_POSES", "failed": "failed"},
            )

            smach.StateMachine.add(
                "GET_POSES", GetPoses(), transitions={"succeeded": "SURVEY"}
            )

            smach.StateMachine.add(
                "SURVEY",
                Survey((-71.0, 71.0), 10),
                transitions={
                    "customer_found": "CHOOSE_WAVING_PERSON",
                    "customer_not_found": "SURVEY",
                },
                remapping={
                    "hands_up_detections": "hands_up_detections",
                },
            )

            smach.StateMachine.add(
                "CHOOSE_WAVING_PERSON",
                ChooseWavingPerson(),
                transitions={
                    "succeeded": "COMPUTE_APPROACH_POSE",
                    "failed": "SURVEY",
                },
                remapping={
                    "hands_up_detections": "hands_up_detections",
                    "waving_person_detection": "waving_person_detection",
                },
            )
            smach.StateMachine.add(
                "COMPUTE_APPROACH_POSE",
                ComputeApproach(),
                transitions={
                    "succeeded": "GO_TO_CUSTOMER",
                    "failed": "SURVEY",
                },
                remapping={
                    "customer_approach_pose": "customer_approach_pose",
                },
            )

            smach.StateMachine.add(
                "GO_TO_CUSTOMER",
                GoToLocation(),
                transitions={
                    "succeeded": "TAKE_ORDER",
                    "failed": "GO_TO_CUSTOMER",
                },
                remapping={
                    "location": "customer_approach_pose",
                },
            )

            smach.StateMachine.add(
                "TAKE_ORDER",
                AskAndListen(
                    tts_phrase="Hello, I'm TIAGo. Please say Hi Tiago to me before speaking. What can I get for you today?"
                ),
                remapping={"transcribed_speech": "order_str"},
                transitions={"succeeded": "HANDLE_ORDER", "failed": "failed"},
            )

            smach.StateMachine.add(
                "HANDLE_ORDER",
                HandleOrder(),
                remapping={"customer_transcription": "order_str"},
                transitions={"succeeded": "SAY_ORDER", "failed": "failed"},
            )

            smach.StateMachine.add(
                "SAY_ORDER",
                Say(
                    format_str="I processed your order as: {}. Is this correct?. Please say 'Yes' or 'No'."
                ),
                remapping={"placeholders": "order_str"},
                transitions={
                    "succeeded": "LISTEN_TO_CONFIRMATION",
                    "preempted": "LISTEN_TO_CONFIRMATION",
                    "aborted": "LISTEN_TO_CONFIRMATION",
                },
            )

            smach.StateMachine.add(
                "LISTEN_TO_CONFIRMATION",
                ListenForWakeword(["yes", "no"], timeout=10.0, threshold=0.3),
                transitions={
                    "succeeded": "CHECK_CONFIRMATION",
                    "failed": "CHECK_CONFIRMATION",  # TODO: handle failure properly
                },
            )

            smach.StateMachine.add(
                "CHECK_CONFIRMATION",
                smach.CBState(
                    lambda ud: ud.keyword == "yes",
                    input_keys=["keyword"],
                    outcomes=["succeeded", "failed"],
                ),
                transitions={"succeeded": "GO_TO_BAR", "failed": "RETAKE_ORDER"},
            )

            smach.StateMachine.add(
                "RETAKE_ORDER",
                AskAndListen(
                    tts_phrase="Let's try again. Please say Hi Tiago to me before speaking. What can I get for you today?"
                ),
                remapping={"transcribed_speech": "order_str"},
                transitions={"succeeded": "HANDLE_RETRY_ORDER", "failed": "failed"},
            )

            smach.StateMachine.add(
                "HANDLE_RETRY_ORDER",
                HandleOrder(),
                remapping={"customer_transcription": "order_str"},
                transitions={"succeeded": "SAY_ORDER_RETRY", "failed": "failed"},
            )

            smach.StateMachine.add(
                "SAY_ORDER_RETRY",
                Say(
                    format_str="I processed your order as: {}. I hope this is now correct. I will go to the bar to get it for you."
                ),
                remapping={"placeholders": "order_str"},
                transitions={
                    "succeeded": "GO_TO_BAR",
                    "preempted": "GO_TO_BAR",
                    "aborted": "GO_TO_BAR",
                },
            )

            smach.StateMachine.add(
                "GO_TO_BAR",
                GoToLocation(),
                remapping={"location": "bar_pose"},
                transitions={"succeeded": "REQUEST_ITEMS", "failed": "failed"},
            )

            smach.StateMachine.add(
                "REQUEST_ITEMS",
                Say(
                    format_str="Please get me {}, I will turn around for you to load the order."
                ),
                remapping={"placeholders": "order_str"},
                transitions={
                    "succeeded": "ROTATE_LOAD",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "ROTATE_LOAD",
                Rotate(angle=180.0),
                transitions={"succeeded": "WAIT_LOAD", "failed": "failed"},
            )

            smach.StateMachine.add(
                "WAIT_LOAD",
                Wait(5),
                transitions={"succeeded": "RETURN_TO_CUSTOMER", "failed": "failed"},
            )

            smach.StateMachine.add(
                "RETURN_TO_CUSTOMER",
                GoToLocation(),
                remapping={"location": "customer_approach_pose"},
                transitions={
                    "succeeded": "SAY_TAKE_ORDER",
                    "failed": "RETURN_TO_CUSTOMER",
                },
            )

            smach.StateMachine.add(
                "SAY_TAKE_ORDER",
                Say(
                    text="Here is your order, I will turn around for you to unload it."
                ),
                transitions={
                    "succeeded": "ROTATE_UNLOAD",
                    "aborted": "ROTATE_UNLOAD",
                    "preempted": "ROTATE_UNLOAD",
                },
            )

            smach.StateMachine.add(
                "ROTATE_UNLOAD",
                Rotate(angle=180.0),
                transitions={"succeeded": "WAIT_UNLOAD", "failed": "ROTATE_UNLOAD"},
            )

            smach.StateMachine.add(
                "WAIT_UNLOAD",
                Wait(5),
                transitions={"succeeded": "GO_TO_SURVEY", "failed": "failed"},
            )

            smach.StateMachine.add(
                "GO_TO_SURVEY",
                GoToLocation(),
                remapping={"location": "survey_pose"},
                transitions={"succeeded": "SURVEY", "failed": "failed"},
            )
