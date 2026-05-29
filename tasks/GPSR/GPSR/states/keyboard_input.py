import smach


class KeyboardInputState(smach.State):
    """Read a voice command from stdin instead of the microphone action server."""

    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "aborted", "preempted"],
            output_keys=["sequence"],
        )
        self.node = node

    def execute(self, userdata):
        prompt = self.node.get_parameter("input_prompt").value
        self.node.get_logger().info(f"Keyboard input mode — {prompt}")

        try:
            text = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            self.node.get_logger().warn("Keyboard input cancelled")
            return "aborted"

        if not text:
            self.node.get_logger().warn("Empty keyboard input")
            return "aborted"

        userdata.sequence = text
        self.node.get_logger().info(f"Command received: '{text}'")
        return "succeeded"
