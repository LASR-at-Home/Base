import yasmin
from GPSR.tts import say


class AnnouncePlan(yasmin.State):
    """Say the plan description before executing it."""

    def __init__(self, node):
        super().__init__(outcomes=["succeeded"])
        self.add_input_key("plan_description")
        self.add_input_key("steps")
        self.add_output_key("plan_description")
        self.add_output_key("steps")
        self.node = node

    def execute(self, blackboard):
        desc = blackboard["plan_description"]
        if not desc:
            return "succeeded"
        self.node.get_logger().info(f"Announcing plan: {desc}")
        say(self.node, desc)
        return "succeeded"
