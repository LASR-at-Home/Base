import json
import time

import yasmin

from GPSR.agent import Agent
from GPSR.prompts import SKILL_SELECTOR_PROMPT, SKILL_REFINER_PROMPT, PLANNER_PROMPT, parse_json as _parse_json
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

        # Stage 2 — skill refiner
        try:
            raw = self.agent.query_json(
                SKILL_REFINER_PROMPT.format(
                    command=command,
                    selected_skills=json.dumps(skills),
                ),
                max_tokens=256,
            )
            refined = _parse_json(raw)
            skills = refined.get("refined_skills", skills)
            self.node.get_logger().info(f"Refined skills: {skills}")
        except Exception as e:
            self.node.get_logger().warn(f"Skill refiner failed, using raw skills: {e}")

        # Stage 3 — planner
        try:
            raw = self.agent.query_json(
                PLANNER_PROMPT.format(
                    general_knowledge=self.general_knowledge,
                    selected_skill_lines=selected_skill_lines(skills, self.skill_lines),
                    selected_skill_names=", ".join(skills),
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
