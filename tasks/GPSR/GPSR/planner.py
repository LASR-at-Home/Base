"""Skill selector → refiner → planner pipeline."""

import json
import time

from GPSR.prompts import (
    ANNOUNCE_PLAN_PROMPT,
    PLANNER_PROMPT,
    SKILL_REFINER_PROMPT,
    SKILL_SELECTOR_PROMPT,
    parse_json as _parse_json,
)
from GPSR.world import format_objects, format_people, selected_skill_lines


class SkillSelectorError(Exception):
    """Stage 1 failed — no plan available."""


def _fail_safe(text: str = "I could not generate a plan for that command.") -> dict:
    return {
        "skill": "say",
        "skill_args": {"text": text},
        "plan_description": text,
        "steps": [{"skill": "say", "args": {"text": text}}],
    }


def run_planner(backend, world: dict, command: str) -> dict:
    """Run the full pipeline using a backend with query_json()."""
    t0 = time.perf_counter()
    command = command.strip()
    # Retrive skill lines from world
    skill_lines = world["skill_lines"]
    try:
        raw = backend.query_json(
            SKILL_SELECTOR_PROMPT.format(skill_lines=skill_lines, command=command),
        )
        selection = _parse_json(raw)
    except Exception as e:
        raise SkillSelectorError(str(e)) from e

    can_do = selection.get("can_do", True)
    reason = selection.get("reason", "")
    skills = selection.get("selected_skills", [])

    # If the command cannot be done, return a fail safe USE LLM TO DECIDE IF THE COMMAND CAN BE DONE
    if not can_do:
        result = _fail_safe(reason or "I'm sorry, I don't know how to do that.")
        result["elapsed_sec"] = round(time.perf_counter() - t0, 2)
        return result
    # Correct the skills using the LLM making sure that the skills respect the rules and respect the style
    try:
        raw = backend.query_json(
            SKILL_REFINER_PROMPT.format(
                command=command,
                selected_skills=json.dumps(skills),
            ),
        )
        refined = _parse_json(raw)
        skills = refined.get("refined_skills", skills)
    except Exception:
        pass

    try:
        raw = backend.query_json(
            PLANNER_PROMPT.format(
                general_knowledge=world["general_knowledge"],
                selected_skill_lines=selected_skill_lines(skills, skill_lines),
                selected_skill_names=", ".join(skills),
                locations=", ".join(world["locations"].keys()) or "none",
                objects=format_objects(world["objects"]),
                people=format_people(world["people"]),
                command=command,
            ),
        )
        parsed = _parse_json(raw)
        steps = parsed.get("steps", [])
        plan_description = parsed.get("plan_description", "")
        if not steps:
            raise ValueError("empty steps")
    except Exception:
        result = _fail_safe()
        result["elapsed_sec"] = round(time.perf_counter() - t0, 2)
        return result

    return {
        "skill": steps[0]["skill"],
        "skill_args": steps[0].get("args", {}),
        "plan_description": plan_description,
        "steps": steps,
        "elapsed_sec": round(time.perf_counter() - t0, 2),
    }


def run_announce(backend, command: str, plan_description: str, steps: list) -> str:
    """One LLM call: turn a plan into a spoken announcement sentence."""
    raw = backend.query_json(
        ANNOUNCE_PLAN_PROMPT.format(
            command=command,
            plan_description=plan_description,
            steps_json=json.dumps(steps),
        ),
    )
    parsed = _parse_json(raw)
    return (parsed.get("announcement") or "").strip()
