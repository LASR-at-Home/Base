import smach_ros
from lasr_speech_recognition_interfaces.action import TranscribeSpeech


class ListenState(smach_ros.SimpleActionState):
    """SMACH state that calls the `transcribe_speech` action and stores the result.

    The transcribed phrase is written to userdata under the `sequence` key.
    Outcomes ("succeeded", "aborted", "preempted") come from the action's
    final status — nothing is decided in this class.
    """

    def __init__(self, node):
        super().__init__(
            node,
            "transcribe_speech",
            TranscribeSpeech,
            result_slots=["sequence"],
        )
