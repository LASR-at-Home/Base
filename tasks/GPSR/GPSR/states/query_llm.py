import json
import time

import yasmin

from GPSR.agent import Agent
from GPSR.world import (
    compact_skill_lines,
    format_objects,
    format_people,
    load_general_knowledge,
    load_locations,
    load_objects,
    load_people,
    load_skills_text,
    selected_skill_lines,
)

SKILL_SELECTOR_PROMPT = """You are the skill selector for a robot.
Given a command, pick which skills from the list below are needed to execute it.
Output ONLY JSON. No explanation.

Available skills:
{skill_lines}

Output schema:
{{
  "can_do": true,
  "reason": "<one sentence why>",
  "selected_skills": ["skill_name", ...]
}}

If no skill can handle the command: can_do=false, selected_skills=[].
Any question or request for information: can_do=true, selected_skills=["say"].

Command: {command}
JSON: """

PLANNER_PROMPT = """You are a robot planner.
General knowledge: {general_knowledge}
Produce a JSON plan using ONLY the selected skills and known world below.

Selected skills:
{selected_skill_lines}

Known locations: {locations}
Known objects: {objects}
Known people: {people}

Rules:
- Use ONLY the selected skills.
- If the command needs a location/object/person NOT in the known lists, emit a single say step refusing politely.
- For say steps: "text" is only the spoken words.
- Output ONE JSON: {{"plan_description": "...", "steps": [{{"skill": "...", "args": {{...}}}}]}}

EXAMPLES:

Command: go to the kitchen
Selected: go_to_location(location) — navigate to a room
Known locations: kitchen, living room, bedroom
Plan: {{"plan_description": "I will go to the kitchen.", "steps": [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}]}}

Command: go to the bathroom
Selected: go_to_location(location) — navigate to a room
Known locations: kitchen, living room, bedroom
Plan: {{"plan_description": "I cannot go to the bathroom.", "steps": [{{"skill": "say", "args": {{"text": "I'm sorry, the bathroom is not a place I know."}}}}]}}

Command: find the apple in the kitchen and bring it to emma
Selected: go_to_location(location), find_object(object, location), pick_up(object), give_to_person(object, name)
Known locations: kitchen, living room, bedroom | Known objects: apple (fruit, kitchen) | Known people: emma
Plan: {{"plan_description": "I will fetch the apple and give it to Emma.", "steps": [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}, {{"skill": "find_object", "args": {{"object": "apple", "location": "kitchen"}}}}, {{"skill": "pick_up", "args": {{"object": "apple"}}}}, {{"skill": "give_to_person", "args": {{"object": "apple", "name": "emma"}}}}]}}

Command: locate the standing person and say hi
Selected: find_person(description), say(text)
Known locations: — | Known objects: — | Known people: - 
Plan: {{"plan_description": "I will find the standing person and say hi.", "
steps": [{{"skill": "find_person", "args": {{"description": "standing person"}}}}, {{"skill": "say", "args": {{"text": "Hi there!"}}}}]}}

HINT: 
[standing person is a person who is standing and locate is an action that means to find where someone is]
locate or find means that the robot shoudl find something or someone in the environment so you cannot produce something telling taht is not in the known list because the robot can find it using its sensors with the right action.

Command: 

Command: what is your team affiliation?
Selected: say(text) — speak aloud
Known locations: — | Known objects: — | Known people: —
Plan: {{"plan_description": "I will state my affiliation.", "steps": [{{"skill": "say", "args": {{"text": "I am from King's College London."}}}}]}}

Command: {command}
Selected: {selected_skill_lines}
Known locations: {locations} | Known objects: {objects} | Known people: {people}
Plan: """


def _parse_json(raw: str) -> dict:
    start = raw.find("{")
    end = raw.rfind("}") + 1
    if start == -1 or end == 0:
        return {}
    return json.loads(raw[start:end])


class QueryLLM(yasmin.State):
    """Two-stage pipeline: skill selector → planner."""

    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("sequence")
        self.add_output_key("skill")
        self.add_output_key("skill_args")
        self.add_output_key("plan_description")
        self.add_output_key("steps")
        self.node = node

        self.locations = load_locations(node)
        self.objects = load_objects(node)
        self.people = load_people(node)
        self.general_knowledge = load_general_knowledge(node)
        skills_text = load_skills_text(node)
        self.skill_lines = compact_skill_lines(skills_text)

        model = node.get_parameter("llm_model").value
        host = node.get_parameter("llm_host").value
        self.agent = Agent(model=model, host=host)

        self.node.get_logger().info(f"Loading model: {model}")
        self.agent.warmup()
        self.node.get_logger().info("QueryLLM ready (skill selector + planner).")

    def _fail_safe(self, blackboard, text="I could not generate a plan for that command."):
        blackboard["skill"] = "say"
        blackboard["skill_args"] = {"text": text}
        blackboard["plan_description"] = text
        blackboard["steps"] = [{"skill": "say", "args": {"text": text}}]

    def execute(self, blackboard):
        t0 = time.perf_counter()
        command = blackboard["sequence"].strip()
        self.node.get_logger().info(f"Query: '{command}'")

        # Stage 1 — skill selector
        try:
            raw = self.agent.query_json(
                SKILL_SELECTOR_PROMPT.format(skill_lines=self.skill_lines, command=command),
                max_tokens=256,
            )
            selection = _parse_json(raw)
            can_do = selection.get("can_do", True)
            reason = selection.get("reason", "")
            skills = selection.get("selected_skills", [])
            self.node.get_logger().info(f"Skills: {skills} | can_do={can_do}")
        except Exception as e:
            self.node.get_logger().error(f"Skill selector failed: {e}")
            return "failed"

        if not can_do:
            self._fail_safe(blackboard, reason or "I'm sorry, I don't know how to do that.")
            return "succeeded"

        # Stage 2 — planner
        try:
            raw = self.agent.query_json(
                PLANNER_PROMPT.format(
                    general_knowledge=self.general_knowledge,
                    selected_skill_lines=selected_skill_lines(skills, self.skill_lines),
                    locations=", ".join(self.locations.keys()) or "none",
                    objects=format_objects(self.objects),
                    people=format_people(self.people),
                    command=command,
                ),
                max_tokens=1024,
            )
            parsed = _parse_json(raw)
            steps = parsed.get("steps", [])
            plan_description = parsed.get("plan_description", "")
            if not steps:
                raise ValueError("empty steps")
        except Exception as e:
            self.node.get_logger().warn(f"Planner failed: {e}")
            self._fail_safe(blackboard)
            return "succeeded"

        blackboard["plan_description"] = plan_description
        blackboard["steps"] = steps
        blackboard["skill"] = steps[0]["skill"]
        blackboard["skill_args"] = steps[0].get("args", {})
        self.node.get_logger().info(
            f"Plan: {plan_description} | {len(steps)} steps | {time.perf_counter()-t0:.1f}s"
        )
        return "succeeded"
