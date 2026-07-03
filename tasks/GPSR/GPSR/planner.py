"""Skill selector → refiner → planner pipeline."""

import json
import time

from GPSR.prompts import (
    ANNOUNCE_PLAN_PROMPT,
    PLANNER_PROMPT,
    SKILL_REFINER_PROMPT,
    SKILL_SELECTOR_PROMPT,
    TRANSCRIPTION_CLEANER_PROMPT,
    parse_json as _parse_json,
)
from GPSR.world import format_objects, format_people, selected_skill_lines, SUBLOCATION_ROOM, placeable_locations


class SkillSelectorError(Exception):
    """Stage 1 failed — no plan available."""


def _inject_sublocations(steps: list, objects: dict) -> list:
    """For every go_to_location that targets a sub-location, prepend a go_to_location for its room.

    Also corrects find_object location args to match the object's known sub-location from world data,
    overriding whatever location the LLM may have hallucinated.
    """
    # Build object → sub-location lookup (both snake_case and space-separated)
    obj_subloc = {}
    for name, obj in objects.items():
        obj_subloc[name] = obj.get("location")
        obj_subloc[name.replace("_", " ")] = obj.get("location")

    def _prev_loc(result):
        """Return the location arg of the last go_to_location step, or None."""
        for step in reversed(result):
            if step.get("skill") == "go_to_location":
                return step.get("args", {}).get("location")
        return None

    result = []
    for step in steps:
        skill = step.get("skill")
        args  = step.get("args", {})

        # Correct find_object location to the object's actual sub-location.
        # The LLM sometimes uses "name" instead of "object" as the arg key, so check both.
        if skill == "find_object":
            obj_name = args.get("object") or args.get("name") or ""
            known_subloc = obj_subloc.get(obj_name)
            if known_subloc:
                args = {**args, "location": known_subloc}
                step = {**step, "args": args}

        # For any go_to_location pointing at a known sub-location,
        # ensure the parent room is visited first.
        if skill == "go_to_location":
            target = args.get("location", "")
            room = SUBLOCATION_ROOM.get(target)
            if room:
                prev = _prev_loc(result)
                if prev != room and prev != target:
                    result.append({"skill": "go_to_location", "args": {"location": room}})

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


def clean_transcription(backend, world: dict, raw_text: str) -> tuple[str, str]:
    """Stage 0: strip preamble noise before the actual command. Returns (cleaned, original)."""
    try:
        raw = backend.query_json(
            TRANSCRIPTION_CLEANER_PROMPT.format(
                transcription=json.dumps(raw_text),
            )
        )
        cleaned = _parse_json(raw).get("cleaned", raw_text).strip()
        if not cleaned:
            cleaned = raw_text
    except Exception:
        cleaned = raw_text
    return cleaned, raw_text


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
        placement = world.get("placement_locations") or placeable_locations(world["locations"])
        raw = backend.query_json(
            PLANNER_PROMPT.format(
                general_knowledge=world["general_knowledge"],
                selected_skill_lines=selected_skill_lines(skills, skill_lines),
                selected_skill_names=", ".join(skills),
                locations=", ".join(world["locations"].keys()) or "none",
                placement_locations=", ".join(placement) or "none",
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
    steps_json = json.dumps(steps).replace("'", "")
    raw = backend.query_json(
        ANNOUNCE_PLAN_PROMPT.format(
            command=command,
            plan_description=plan_description,
            steps_json=steps_json,
        ),
    )
    parsed = _parse_json(raw)
    return (parsed.get("announcement") or "").strip()
