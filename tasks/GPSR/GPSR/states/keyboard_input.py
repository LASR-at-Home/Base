import yasmin
import yasmin_ros
INPUT_PROMPT = "Enter command: "


class KeyboardInputState(yasmin.State):
    """Read a command from keyboard instead of the microphone action server."""

    def __init__(self):
        super().__init__(outcomes=["succeeded", "aborted"])
        self.add_output_key("transcribed_speech")
        self.node = yasmin_ros.logger_node

    def execute(self, blackboard):
        prompt = INPUT_PROMPT
        yasmin.YASMIN_LOG_INFO(f"Keyboard input mode — {prompt}")

        try:
            text = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            yasmin.YASMIN_LOG_WARN("Keyboard input cancelled")
            return "aborted"

        if not text:
            yasmin.YASMIN_LOG_WARN("Empty keyboard input")
            return "aborted"

        blackboard["transcribed_speech"] = text
        yasmin.YASMIN_LOG_INFO(f"Command received: '{text}'")
        return "succeeded"
