import json
import os

import yasmin
import yaml
from ament_index_python.packages import get_package_share_directory

from GPSR.agent import Agent

SYSTEM_PROMPT = """You are a robot assistant. Given a voice command, output ONLY a JSON object — no explanation, no extra text.

Available skills:
- go_to_location: navigate to a named location. Args: {{"location": "<name>"}}
- say: speak a sentence. Args: {{"text": "<sentence>"}}
- find_object: find an object in a location. Args: {{"object": "<name>", "location": "<name>"}}
- pick_up: pick up an object. Args: {{"object": "<name>"}}
- give_to_person: give an object to a person. Args: {{"object": "<name>"}}

Known locations: {locations}

Output format:
{{
  "plan_description": "<natural language description of the full plan, spoken in first person as the robot>",
  "steps": [
    {{"skill": "<skill_name>", "args": {{...}}}},
    ...
  ]
}}

Examples:
  "go to the kitchen" -> {{"plan_description": "I will navigate to the kitchen.", "steps": [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}]}}
  "find a pear in the bathroom then fetch it and bring it to the person raising their right arm in the bedroom" -> {{"plan_description": "I will go to the bathroom to find the pear, pick it up, then go to the bedroom and give it to the person raising their right arm.", "steps": [{{"skill": "go_to_location", "args": {{"location": "bathroom"}}}}, {{"skill": "find_object", "args": {{"object": "pear", "location": "bathroom"}}}}, {{"skill": "pick_up", "args": {{"object": "pear"}}}}, {{"skill": "go_to_location", "args": {{"location": "bedroom"}}}}, {{"skill": "give_to_person", "args": {{"object": "pear"}}}}]}}

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


class QueryLLM(yasmin.State):
    """YASMIN state that turns a transcribed phrase into a structured skill call."""

    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("sequence")
        self.add_output_key("skill")
        self.add_output_key("skill_args")
        self.add_output_key("plan_description")
        self.add_output_key("steps")
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

    def execute(self, blackboard):
        command = blackboard["sequence"].strip()
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
            steps = parsed.get("steps", [])
            plan_description = parsed.get("plan_description", "")
            if not steps:
                raise ValueError("No steps in plan")
        except (ValueError, KeyError, json.JSONDecodeError) as e:
            self.node.get_logger().warn(f"LLM parse error: {e}")
            blackboard["skill"] = "say"
            blackboard["skill_args"] = {"text": "I did not understand that command"}
            blackboard["plan_description"] = ""
            blackboard["steps"] = []
            return "succeeded"

        blackboard["plan_description"] = plan_description
        blackboard["steps"] = steps
        blackboard["skill"] = steps[0]["skill"]
        blackboard["skill_args"] = steps[0].get("args", {})
        self.node.get_logger().info(f"Plan: {plan_description}, Steps: {steps}")
        return "succeeded"
