import smach
from geometry_msgs.msg import Point, Pose, Quaternion

from GPSR.states.query_llm import load_locations
from lasr_skills import GoToLocation, Say


class DispatchSkill(smach.State):
    """SMACH state that executes a skill chosen by the LLM."""

    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["skill", "skill_args"],
        )
        self.node = node
        self.locations = load_locations(node)

    def _say(self, text):
        if not text:
            return "succeeded"
        self.node.get_logger().info(f"Saying: {text}")
        outcome = Say(node=self.node, text=text).execute({})
        if outcome != "succeeded":
            self.node.get_logger().warn(f"Say skill finished with outcome: {outcome}")
        return "succeeded" if outcome == "succeeded" else "failed"

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
        state = GoToLocation(node=self.node, location=pose)
        return state.execute(userdata={})

    def execute(self, userdata):
        skill = userdata.skill
        args = userdata.skill_args

        if skill == "say":
            return self._say(args.get("text", ""))
        if skill == "go_to_location":
            return self._go_to_location(args.get("location", ""))

        self.node.get_logger().warn(f"Unknown skill: {skill}")
        self._say(f"I don't know how to {skill}")
        return "failed"
