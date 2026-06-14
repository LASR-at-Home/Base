import yasmin
from geometry_msgs.msg import Point, Pose, Quaternion

from GPSR.world import load_locations
from GPSR.tts import say
from lasr_skills import GoToLocation


class DispatchSkill(yasmin.State):
    """YASMIN state that executes a skill chosen by the LLM."""

    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("steps")
        self.node = node
        self.locations = load_locations(node)

    def _say(self, text):
        if not text:
            return "succeeded"
        self.node.get_logger().info(f"Saying: {text}")
        say(self.node, text)
        return "succeeded"

    def _go_to_location(self, location_name):
        if location_name not in self.locations:
            self.node.get_logger().error(f"Unknown location: {location_name}")
            self._say(f"I don't know where {location_name} is")
            return "failed"
        loc = self.locations[location_name]
        pose = Pose(
            position=Point(
                x=float(loc["position"]["x"]),
                y=float(loc["position"]["y"]),
                z=float(loc["position"].get("z", 0.0)),
            ),
            orientation=Quaternion(
                x=float(loc["orientation"]["x"]),
                y=float(loc["orientation"]["y"]),
                z=float(loc["orientation"]["z"]),
                w=float(loc["orientation"]["w"]),
            ),
        )
        self.node.get_logger().info(f"Navigating to '{location_name}'")
        bb = yasmin.Blackboard()
        return GoToLocation(location=pose)(bb)

    def _execute_step(self, skill, args):
        if skill == "say":
            return self._say(args.get("text", ""))
        #TODO: Add other skills
        """
        example: 
        if skill == "go_to_location":
            return self._go_to_location(args.get("location", ""))
        """
        self.node.get_logger().info(f"Skipping skill '{skill}' (not yet actuated)")
        return "succeeded"

    def execute(self, blackboard):
        for step in blackboard["steps"]:
            outcome = self._execute_step(step["skill"], step.get("args", {}))
            if outcome == "failed":
                return "failed"
        return "succeeded"
