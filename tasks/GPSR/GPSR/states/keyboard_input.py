import sys
import time

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
        # Under `ros2 launch` the node has no terminal, so input() fails at once and
        # this state would spin back to itself forever.
        if not sys.stdin or not sys.stdin.isatty():
            yasmin.YASMIN_LOG_ERROR(
                "Keyboard input mode needs a terminal: start the node with "
                "`ros2 run GPSR service` (not ros2 launch) and type in that terminal, "
                'or set input_mode to "microphone".'
            )
            time.sleep(5.0)
            return "aborted"

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
