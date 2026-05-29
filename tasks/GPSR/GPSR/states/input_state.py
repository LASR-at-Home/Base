from GPSR.states.keyboard_input import KeyboardInputState
from GPSR.states.listen import ListenState


def create_input_state(node):
    """Return the input state configured by the `input_mode` parameter."""
    mode = node.get_parameter("input_mode").value.strip().lower()

    if mode == "keyboard":
        return KeyboardInputState(node)
    if mode in ("mic", "microphone"):
        return ListenState(node)

    node.get_logger().warn(
        f"Unknown input_mode '{mode}', falling back to microphone"
    )
    return ListenState(node)
