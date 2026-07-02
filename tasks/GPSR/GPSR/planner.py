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
from GPSR.world import format_objects, format_people, selected_skill_lines, SUBLOCATION_ROOM


class SkillSelectorError(Exception):
    """Stage 1 failed — no plan available."""


def _inject_sublocations(steps: list, objects: dict) -> list:
    """Ensure find_object steps are preceded by go_to_location(room) then go_to_location(sub-location)."""
    # Build object→sublocation lookup
    obj_subloc = {name: obj.get("location") for name, obj in objects.items()}

    result = []
    for step in steps:
        if step.get("skill") == "find_object":
            obj_name = step.get("args", {}).get("object", "")
            subloc = obj_subloc.get(obj_name)
            if subloc:
                room = SUBLOCATION_ROOM.get(subloc)
                # Check what the previous step navigated to
                prev_loc = result[-1].get("args", {}).get("location") if result and result[-1].get("skill") == "go_to_location" else None
                # Inject room nav if not already there
                if room and prev_loc != room and prev_loc != subloc:
                    result.append({"skill": "go_to_location", "args": {"location": room}})
                # Inject sub-location nav if not already there
                if prev_loc != subloc:
                    result.append({"skill": "go_to_location", "args": {"location": subloc}})
                # Update find_object args to use correct sub-location
                step = {**step, "args": {**step.get("args", {}), "location": subloc}}
        result.append(step)
    return result


def _inject_give_to_operator(steps: list) -> list:
    """If the plan picks up an object but doesn't end with give_to_person, add return + give."""
    skills = [s.get("skill") for s in steps]
    if "pick_up" not in skills:
        return steps
    if "give_to_person" in skills:
        return steps
    # Missing the delivery — append return to instruction point and give to operator
    steps = list(steps)
    last_loc = steps[-1].get("args", {}).get("location") if steps[-1].get("skill") == "go_to_location" else None
    if last_loc != "instruction point":
        steps.append({"skill": "go_to_location", "args": {"location": "instruction point"}})
    steps.append({"skill": "give_to_person", "args": {"person": "operator"}})
    return steps


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
        steps = _inject_sublocations(steps, world["objects"])
        steps = _inject_give_to_operator(steps)
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
