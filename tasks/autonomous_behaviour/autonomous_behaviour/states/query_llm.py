import json
from pathlib import Path

import smach
import yaml

from autonomous_behaviour.agent import Agent

LOCATIONS_YAML = Path(__file__).parents[4] / "common" / "simulation" / "maps" / "locations.yaml"

SYSTEM_PROMPT = """You are a robot assistant. Given a voice command, output ONLY a JSON object — no explanation, no extra text.

Available skills:
- go_to_location: navigate to a named location. Args: {{"location": "<name>"}}
- say: speak a sentence. Args: {{"text": "<sentence>"}}

Known locations: {locations}

Examples:
  "go to the kitchen" -> {{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}
  "say hello" -> {{"skill": "say", "args": {{"text": "hello"}}}}

Command: """


def load_locations():
    """Load named navigation goals from `locations.yaml`. Returns {} if missing."""
    if not LOCATIONS_YAML.exists():
        return {}
    with open(LOCATIONS_YAML) as f:
        data = yaml.safe_load(f) or {}
    return data.get("locations", {})


class QueryLLM(smach.State):
    """SMACH state that turns a transcribed phrase into a structured skill call.

    Sends the user's command to an LLM agent and parses its JSON reply into
    `skill` and `skill_args` userdata fields.
    """

    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["sequence"],
            output_keys=["skill", "skill_args"],
        )
        self.node = node
        locations = load_locations()
        location_names = list(locations.keys()) if locations else []
        self.system_prompt = SYSTEM_PROMPT.format(locations=", ".join(location_names))

        self.node.get_logger().info("Loading agent (Ollama)...")
        self.agent = Agent(system_prompt=self.system_prompt)
        self.node.get_logger().info("Agent ready.")

    def execute(self, userdata):
        """Query the LLM, parse the JSON reply, and write skill/args to userdata.

        Returns:
            "failed"    on agent/network errors (no spoken feedback).
            "succeeded" on a valid parse, or with a 'say' fallback when the
                        JSON is malformed.
        """
        command = userdata.sequence.strip()
        self.node.get_logger().info(f"LLM query: '{command}'")

        try:
            raw = self.agent.query_json(command)
        except Exception as e:
            self.node.get_logger().error(f"Agent query failed: {e}")
            return "failed"

        self.node.get_logger().info(f"LLM raw output: '{raw}'")

        try:
            start = raw.find("{")
            end = raw.rfind("}") + 1
            if start == -1 or end == 0:
                raise ValueError("No JSON found in LLM output")
            parsed = json.loads(raw[start:end])
            skill = parsed["skill"]
            skill_args = parsed.get("args", {})
        except (ValueError, KeyError, json.JSONDecodeError) as e:
            self.node.get_logger().warn(f"LLM parse error: {e}")
            userdata.skill = "say"
            userdata.skill_args = {"text": "I did not understand that command"}
            return "succeeded"

        userdata.skill = skill
        userdata.skill_args = skill_args
        self.node.get_logger().info(f"Skill: {skill}, Args: {skill_args}")
        return "succeeded"
