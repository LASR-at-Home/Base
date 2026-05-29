import json
import os

import smach
import yaml
from ament_index_python.packages import get_package_share_directory

from GPSR.agent import Agent

SYSTEM_PROMPT = """You are a robot assistant. Given a voice command, output ONLY a JSON object — no explanation, no extra text.

Available skills:
- go_to_location: navigate to a named location. Args: {{"location": "<name>"}}
- say: speak a sentence. Args: {{"text": "<sentence>"}}

Known locations: {locations}

Examples:
  "go to the kitchen" -> {{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}
  "say hello" -> {{"skill": "say", "args": {{"text": "hello"}}}}

Command: """


def load_locations(node):
    """Load named navigation goals from the configured locations file."""
    package = node.get_parameter("locations_package").value
    locations_file = node.get_parameter("locations_file").value
    locations_path = os.path.join(get_package_share_directory(package), locations_file)

    if not os.path.exists(locations_path):
        node.get_logger().warn(f"Locations file not found: {locations_path}")
        return {}

    with open(locations_path) as f:
        data = yaml.safe_load(f) or {}
    return data.get("locations", {})


class QueryLLM(smach.State):
    """SMACH state that turns a transcribed phrase into a structured skill call."""

    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["sequence"],
            output_keys=["skill", "skill_args"],
        )
        self.node = node
        locations = load_locations(node)
        location_names = list(locations.keys()) if locations else []
        self.system_prompt = SYSTEM_PROMPT.format(locations=", ".join(location_names))

        self.node.get_logger().info("Loading agent (Ollama)...")
        self.agent = Agent(
            model=node.get_parameter("llm_model").value,
            system_prompt=self.system_prompt,
            host=node.get_parameter("llm_host").value,
        )
        self.node.get_logger().info("Agent ready.")

    def execute(self, userdata):
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
