import yasmin

INPUT_PROMPT = "Enter command: "


class KeyboardInputState(yasmin.State):
    """Read a command from keyboard instead of the microphone action server."""

    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "aborted"])
        self.add_output_key("sequence")
        self.node = node

    def execute(self, blackboard):
        prompt = INPUT_PROMPT
        self.node.get_logger().info(f"Keyboard input mode — {prompt}")

        try:
            text = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            self.node.get_logger().warn("Keyboard input cancelled")
            return "aborted"

        if not text:
            self.node.get_logger().warn("Empty keyboard input")
            return "aborted"

        blackboard["sequence"] = text
        self.node.get_logger().info(f"Command received: '{text}'")
        return "succeeded"
