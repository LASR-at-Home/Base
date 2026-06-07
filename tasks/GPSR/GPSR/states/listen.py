import yasmin
import yasmin_ros
from lasr_speech_recognition_interfaces.action import TranscribeSpeech


class ListenState(yasmin_ros.ActionState):
    """YASMIN state that calls the `transcribe_speech` action and stores the result."""

    def __init__(self, node):
        super().__init__(
            TranscribeSpeech,
            "/transcribe_speech",
            self._create_goal,
            outcomes=["succeeded", "aborted"],
            result_handler=self._handle_result,
        )
        self.add_output_key("sequence")

    def _create_goal(self, blackboard):
        return TranscribeSpeech.Goal()

    def _handle_result(self, blackboard, result):
        blackboard["sequence"] = result.sequence
        return "succeeded"
