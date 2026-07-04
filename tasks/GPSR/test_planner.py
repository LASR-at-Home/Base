#!/usr/bin/env python3
"""
Standalone GPSR planner test — zero ROS dependency.

Requirements:
    pip install ollama pyyaml
    ollama must be running locally with the model available.

Usage:
    python test_planner.py                        # run all commands
    python test_planner.py --model gemma3         # choose model
    python test_planner.py --batch 3              # only batch index 3 (0-based)
    python test_planner.py --cmd "bring me the apple"  # single ad-hoc command
    python test_planner.py --no-color             # plain text output
"""

import argparse
import json
import os
import re
import sys
import time
import yaml
from pathlib import Path
from dataclasses import dataclass, field
from typing import Callable, List, Optional

import ollama

# ── Paths ────────────────────────────────────────────────────────────────────
_HERE = Path(__file__).resolve().parent
CONFIG = _HERE / "config"

# ── Prompts (copy of prompts.py, no ROS imports) ─────────────────────────────
# Loaded at runtime from the real prompts.py via exec so they stay in sync.
_prompts_src = (_HERE / "GPSR" / "prompts.py").read_text()
_prompts_ns: dict = {}
exec(compile(_prompts_src, "prompts.py", "exec"), _prompts_ns)

SKILL_SELECTOR_PROMPT        = _prompts_ns["SKILL_SELECTOR_PROMPT"]
SKILL_REFINER_PROMPT         = _prompts_ns["SKILL_REFINER_PROMPT"]
PLANNER_PROMPT               = _prompts_ns["PLANNER_PROMPT"]
TRANSCRIPTION_CLEANER_PROMPT = _prompts_ns["TRANSCRIPTION_CLEANER_PROMPT"]
_parse_json                  = _prompts_ns["parse_json"]

# ── World loader (no ROS) ────────────────────────────────────────────────────
SUBLOCATION_ROOM = {
    # laundry
    "laundry table":     "laundry",
    "washing machine":   "laundry",
    "shelf":             "laundry",
    "laundry trash bin": "laundry",
    # bedroom
    "bed":               "bedroom",
    "bedside table":     "bedroom",
    "coat rack":         "bedroom",
    # living room
    "tv stand":          "living room",
    "sofa":              "living room",
    "coffee table":      "living room",
    # kitchen
    "cabinet":           "kitchen",
    "refrigerator":      "kitchen",
    "counter":           "kitchen",
    "sink":              "kitchen",
    "cooking table":     "kitchen",
    "dishwasher":        "kitchen",
    "kitchen trash bin": "kitchen",
    "dinner table":      "kitchen",
}

TRASH_LOCATIONS = {"laundry trash bin", "kitchen trash bin"}


def _load_yaml(filename: str) -> dict:
    path = CONFIG / filename
    with open(path) as f:
        return yaml.safe_load(f) or {}


def _placeable_locations(locations: dict) -> list:
    return [name for name, info in locations.items() if info.get("placeable", False)]


def load_world() -> dict:
    import datetime
    locations = _load_yaml("locations.yaml").get("locations", {})
    objects   = _load_yaml("objects.yaml").get("objects", {})
    people    = _load_yaml("people.yaml").get("people", {})
    gk        = _load_yaml("general_knowledge.yaml").get("info", "")

    now = datetime.datetime.now()
    gk += (
        f" The current date is {now.strftime('%-d %B %Y')}."
        f" The day of the week is {now.strftime('%A')}."
        f" The day of the month is {now.strftime('%-d')}."
        f" The current time is {now.strftime('%H:%M')}."
        f" Tomorrow is {(now + datetime.timedelta(days=1)).strftime('%A')}."
    )

    # Build compact skill lines from skills.yaml
    skill_lines = []
    for raw in open(CONFIG / "skills.yaml").readlines():
        raw = raw.rstrip()
        if not raw.strip() or raw.startswith("#"):
            continue
        if ":" not in raw:
            continue
        name, desc = raw.split(":", 1)
        name = name.strip().lstrip("- ")
        desc = desc.strip()
        args = ""
        if "Args:" in desc:
            purpose, _, arg_tail = desc.partition("Args:")
            arg_names = []
            for part in arg_tail.split(","):
                token = part.strip().split(" ")[0].strip(" .()")
                if token and token.lower() != "none":
                    arg_names.append(token)
            args = ", ".join(arg_names)
            desc = purpose.strip()
        purpose = desc.split(".")[0].strip()
        skill_lines.append(f"{name}({args}) — {purpose}")

    # Format objects/people for prompt
    obj_parts = []
    for name, obj in objects.items():
        subloc = obj.get("location", "?")
        room = SUBLOCATION_ROOM.get(subloc, "?")
        obj_parts.append(f"{name} ({obj.get('category','?')} at {subloc} in {room})")
    objects_str = ", ".join(obj_parts) or "none"

    people_str = ", ".join(
        f"{n} ({i.get('gender','?')})" for n, i in people.items()
    ) or "none"

    placement = _placeable_locations(locations)

    return {
        "locations":           locations,
        "locations_str":       ", ".join(locations.keys()),
        "placement_locations": placement,
        "placement_str":       ", ".join(placement),
        "objects":             objects,
        "objects_str":         objects_str,
        "people":              people,
        "people_str":          people_str,
        "general_knowledge":   gk,
        "skill_lines":         "\n".join(skill_lines),
    }


def _skill_lines_for(selected: list, all_lines: str) -> str:
    if not selected:
        return "say(text) — speak aloud"
    result = []
    for line in all_lines.splitlines():
        skill_name = line.split("(")[0].strip()
        if skill_name in selected:
            result.append(line)
    return "\n".join(result) if result else all_lines


# ── LLM backend ──────────────────────────────────────────────────────────────
class LocalBackend:
    def __init__(self, model: str, host: str):
        self._model = model
        self._client = ollama.Client(host=host, timeout=300.0)

    def query_json(self, prompt: str) -> str:
        resp = self._client.chat(
            model=self._model,
            messages=[{"role": "user", "content": prompt}],
            format="json",
        )
        return resp["message"]["content"]


# ── Pipeline (mirrors planner.py without any ROS/agent machinery) ─────────────
def _inject_sublocations(steps: list, objects: dict) -> list:
    """For every go_to_location that targets a known sub-location, prepend its parent room."""
    def _prev_loc(result):
        for step in reversed(result):
            if step.get("skill") == "go_to_location":
                return step.get("args", {}).get("location")
        return None

    result = []
    for step in steps:
        skill = step.get("skill")
        args  = step.get("args", {})

        if skill == "go_to_location":
            target = args.get("location", "")
            room = SUBLOCATION_ROOM.get(target)
            if room:
                prev = _prev_loc(result)
                if prev != room and prev != target:
                    result.append({"skill": "go_to_location", "args": {"location": room}})

        result.append(step)
    return result


def _clean_transcription(backend: LocalBackend, world: dict, raw_text: str) -> tuple[str, str]:
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


def run_pipeline(backend: LocalBackend, world: dict, command: str) -> dict:
    """Selector → refiner → planner. Returns plan dict."""
    command = command.strip()

    # Stage 0: transcription cleaner
    command, original = _clean_transcription(backend, world, command)

    # Stage 1: skill selector
    raw = backend.query_json(
        SKILL_SELECTOR_PROMPT.format(
            skill_lines=world["skill_lines"],
            command=command,
        )
    )
    selection = _parse_json(raw)
    can_do = selection.get("can_do", True)
    skills = selection.get("selected_skills", [])

    if not can_do:
        return {
            "plan_description": selection.get("reason", "cannot do"),
            "steps": [{"skill": "say", "args": {"text": selection.get("reason", "I cannot do that.")}}],
            "_selector": selection,
            "_refined_skills": [],
        }

    # Stage 2: refiner
    try:
        raw = backend.query_json(
            SKILL_REFINER_PROMPT.format(
                command=command,
                selected_skills=json.dumps(skills),
            )
        )
        refined = _parse_json(raw)
        skills = refined.get("refined_skills", skills)
    except Exception:
        pass

    # Stage 3: planner
    raw = backend.query_json(
        PLANNER_PROMPT.format(
            general_knowledge=world["general_knowledge"],
            selected_skill_lines=_skill_lines_for(skills, world["skill_lines"]),
            selected_skill_names=", ".join(skills),
            locations=world["locations_str"],
            placement_locations=world["placement_str"],
            objects=world["objects_str"],
            people=world["people_str"],
            command=command,
        )
    )
    parsed = _parse_json(raw)
    steps  = parsed.get("steps", [])
    steps  = _inject_sublocations(steps, world["objects"])

    return {
        "plan_description": parsed.get("plan_description", ""),
        "steps": steps,
        "_selector": selection,
        "_refined_skills": skills,
        "_original_command": original,
        "_cleaned_command": command,
    }


# ── Validation DSL ────────────────────────────────────────────────────────────
@dataclass
class Check:
    label: str
    fn: Callable[[list], bool]
    severity: str = "FAIL"   # FAIL | WARN


def _skills(steps):
    return [s.get("skill") for s in steps]


def _args(steps, skill):
    """All args dicts for steps with the given skill."""
    return [s.get("args", {}) for s in steps if s.get("skill") == skill]


def is_refuse(steps) -> bool:
    return len(steps) == 1 and steps[0].get("skill") == "say"


def has_skill(skill) -> Check:
    return Check(f"has {skill}", lambda s: skill in _skills(s))

def not_skill(skill) -> Check:
    return Check(f"no {skill}", lambda s: skill not in _skills(s))

def ends_with(skill) -> Check:
    return Check(f"ends with {skill}", lambda s: bool(s) and s[-1].get("skill") == skill)

def expect_refuse() -> Check:
    return Check("refuses (single say)", is_refuse)

def expect_no_refuse() -> Check:
    return Check("does not refuse", lambda s: not is_refuse(s))

def delivery_to_me() -> Check:
    def _check(steps):
        if not any(s.get("skill") == "give_to_person" for s in steps):
            return False
        return any(
            "operator" in str(a.get("person", "")).lower()
            for a in _args(steps, "give_to_person")
        )
    return Check("give_to_person → operator", _check)

def delivery_to_person(name: str) -> Check:
    def _check(steps):
        return any(
            name.lower() in str(a.get("person", "")).lower()
            for a in _args(steps, "give_to_person")
        )
    return Check(f"give_to_person → {name}", _check)

def place_at_trash() -> Check:
    def _check(steps):
        return any(
            a.get("location", "") in TRASH_LOCATIONS
            for a in _args(steps, "place_object")
        )
    return Check("place_object → trash bin", _check)

def find_person_with(**kwargs) -> Check:
    desc = ", ".join(f"{k}={v}" for k, v in kwargs.items())
    def _check(steps):
        for a in _args(steps, "find_person"):
            if all(kwargs[k].lower() in str(a.get(k, "")).lower() for k in kwargs):
                return True
        return False
    return Check(f"find_person({desc})", _check)

def guide_person_to(dest: str) -> Check:
    def _check(steps):
        return any(
            dest.lower() in str(a.get("end", "")).lower()
            for a in _args(steps, "guide_person")
        )
    return Check(f"guide_person → {dest}", _check)

def navigates_to(loc: str) -> Check:
    def _check(steps):
        return any(
            str(a.get("location", "")).lower() == loc.lower()
            for a in _args(steps, "go_to_location")
        )
    return Check(f"navigates to {loc}", _check)

def correct_object_nav(obj_name: str, world: dict) -> Check:
    """Object is found at its correct sublocation (from world knowledge)."""
    obj_data = world["objects"].get(obj_name) or world["objects"].get(obj_name.replace(" ", "_"))
    if obj_data is None:
        return Check(f"object '{obj_name}' known", lambda s: False, severity="WARN")
    subloc = obj_data.get("location", "")
    room   = SUBLOCATION_ROOM.get(subloc, "")
    def _check(steps):
        locs = [a.get("location", "").lower() for a in _args(steps, "go_to_location")]
        find_locs = [a.get("location", "").lower() for a in _args(steps, "find_object")]
        return subloc.lower() in locs or subloc.lower() in find_locs
    label = f"navigates to correct subloc ({subloc} in {room})"
    return Check(label, _check, severity="WARN")

def pick_up_followed_by_delivery() -> Check:
    def _check(steps):
        skills = _skills(steps)
        if "pick_up" not in skills:
            return True
        return "give_to_person" in skills or "place_object" in skills
    return Check("pick_up has delivery (give/place)", _check)


# ── Test cases ────────────────────────────────────────────────────────────────
# Each entry: (command_string, [list of Check objects], optional_notes)
#
# KNOWN WORLD:
#   Rooms: bedroom, kitchen, laundry, living room
#   Sub-locations: see SUBLOCATION_ROOM above
#   NOT in locations: bedside table, garden, garage, bathroom
#   NOT in objects:   toothpaste
#   Objects at wrong locations in commands are noted with (*)

def build_test_cases(world: dict) -> List[tuple]:
    return [
        # ── Batch 1 ──────────────────────────────────────────────────────────
        (
            "Navigate to the laundry then look for an instant noodles and take it and put it on the cabinet",
            [has_skill("pick_up"), has_skill("place_object"), pick_up_followed_by_delivery(),
             correct_object_nav("instant_noodles", world)],
            "instant_noodles is at coffee table (living room), not laundry",
        ),
        (
            "Locate a waving person in the bedroom and follow them to the laundry",
            [has_skill("find_person"), has_skill("follow_person"), navigates_to("bedroom")],
            None,
        ),
        (
            "Go to the laundry table then locate a peach and get it and put it on the bedside table",
            [has_skill("pick_up"), has_skill("place_object"), pick_up_followed_by_delivery(),
             navigates_to("bedside table"), correct_object_nav("peach", world)],
            "peach is at dinner table (kitchen); command says laundry table (*)",
        ),
        # ── Batch 2 ──────────────────────────────────────────────────────────
        (
            "Give me a lemon from the laundry table",
            [has_skill("pick_up"), delivery_to_me(), pick_up_followed_by_delivery()],
            "lemon is at dinner table (kitchen); command says laundry table (*)",
        ),
        (
            "Go to the cabinet then locate the person pointing to the left and follow them",
            [has_skill("find_person"), has_skill("follow_person")],
            None,
        ),
        (
            "Meet Simone in the kitchen and follow them to the living room",
            [navigates_to("kitchen"), find_person_with(name="simone"), has_skill("follow_person")],
            None,
        ),
        # ── Batch 3 ──────────────────────────────────────────────────────────
        (
            "Get a coke from the bedside table and throw it in the trash",
            [has_skill("pick_up"), place_at_trash(), pick_up_followed_by_delivery(),
             correct_object_nav("coke", world)],
            "coke is at cabinet (kitchen), not bedside table (*)",
        ),
        (
            "Navigate to the bedside table then locate the standing person and say the time",
            [navigates_to("bedside table"), find_person_with(pose="standing"), has_skill("say")],
            None,
        ),
        (
            "Greet Adel in the bedroom and guide them to the living room",
            [navigates_to("bedroom"), find_person_with(name="adel"),
             has_skill("guide_person"), guide_person_to("living room")],
            None,
        ),
        # ── Batch 4 ──────────────────────────────────────────────────────────
        (
            "Navigate to the bedroom then find a seaweed and grasp it and bring it to Axel in the living room",
            [has_skill("pick_up"), delivery_to_person("axel"), pick_up_followed_by_delivery(),
             correct_object_nav("seaweed", world)],
            "seaweed is at shelf (laundry), not bedroom (*)",
        ),
        (
            "Find a person pointing to the right in the laundry and tell your teams country",
            [navigates_to("laundry"), has_skill("find_person"), has_skill("say")],
            None,
        ),
        (
            "Find a person pointing to the left in the bedroom and say what day is tomorrow",
            [navigates_to("bedroom"), has_skill("find_person"), has_skill("say")],
            None,
        ),
        # ── Batch 5 ──────────────────────────────────────────────────────────
        (
            "Tell me how many people in the kitchen are wearing gray blouses",
            [has_skill("count_people"), ends_with("say")],
            None,
        ),
        (
            "Tell me what is the lightest fabric on the tv stand",
            [has_skill("find_object_by_property"), ends_with("say"), navigates_to("tv stand")],
            None,
        ),
        (
            "Go to the laundry then find a knife and take it and deliver it to me",
            [has_skill("pick_up"), delivery_to_me(), pick_up_followed_by_delivery(),
             correct_object_nav("knife", world)],
            "knife is at dishwasher (kitchen), not laundry (*)",
        ),
        # ── Batch 6 ──────────────────────────────────────────────────────────
        (
            "Take an instant noodles from the cabinet and place it on the dishwasher",
            [has_skill("pick_up"), has_skill("place_object"), pick_up_followed_by_delivery(),
             correct_object_nav("instant_noodles", world)],
            "instant_noodles is at coffee table (living room), not cabinet (*)",
        ),
        (
            "Go to the laundry then find the sitting person and follow them to the refrigerator",
            [find_person_with(pose="sitting"), has_skill("follow_person")],
            None,
        ),
        (
            "Take a cleaning supply from the cabinet and give it to the sitting person in the living room",
            [has_skill("pick_up"), has_skill("find_person"), has_skill("give_to_person"),
             pick_up_followed_by_delivery()],
            "cleaning supplies are at laundry table (laundry), not cabinet (*)",
        ),
        # ── Batch 7 ──────────────────────────────────────────────────────────
        (
            "Go to the kitchen trash bin then locate the standing person and lead them to the cabinet",
            [find_person_with(pose="standing"), has_skill("guide_person"), guide_person_to("cabinet")],
            None,
        ),
        (
            "Tell me what is the biggest object on the bedside table",
            [has_skill("find_object_by_property"), ends_with("say"), navigates_to("bedside table")],
            None,
        ),
        (
            "Locate a sponge in the kitchen then take it and throw it in the trash",
            [has_skill("pick_up"), place_at_trash(), pick_up_followed_by_delivery(),
             correct_object_nav("sponge", world)],
            "sponge is at laundry table (laundry), not kitchen (*)",
        ),
        # ── Batch 8 ──────────────────────────────────────────────────────────
        (
            "Tell me what is the heaviest object on the dishwasher",
            [has_skill("find_object_by_property"), ends_with("say")],
            None,
        ),
        (
            "Navigate to the tv stand then meet Robin and follow them",
            [navigates_to("tv stand"), find_person_with(name="robin"), has_skill("follow_person")],
            None,
        ),
        (
            "Tell me how many snacks there are on the cabinet",
            [has_skill("count_objects"), ends_with("say")],
            "snacks are at shelf (laundry); count_objects should still handle (*)",
        ),
        # ── Batch 9 ──────────────────────────────────────────────────────────
        (
            "Locate a food in the bedroom then fetch it and give it to Adel in the living room",
            [has_skill("pick_up"), delivery_to_person("adel"), pick_up_followed_by_delivery(),
             correct_object_nav("instant_noodles", world)],
            "food (instant_noodles) is at coffee table (living room), not bedroom (*)",
        ),
        (
            "Tell me what is the biggest object on the dishwasher",
            [has_skill("find_object_by_property"), ends_with("say")],
            None,
        ),
        (
            "Go to the tv stand then meet Adel and tell something about yourself",
            [navigates_to("tv stand"), find_person_with(name="adel"), has_skill("say")],
            None,
        ),
        # ── Batch 10 ─────────────────────────────────────────────────────────
        (
            "Navigate to the kitchen then locate a food and grasp it and give it to Angel in the bedroom",
            [has_skill("pick_up"), delivery_to_person("angel"), pick_up_followed_by_delivery()],
            "food (instant_noodles) at coffee table (living room); command says kitchen (*)",
        ),
        (
            "Tell me what is the lightest snack on the coffee table",
            [has_skill("find_object_by_property"), ends_with("say")],
            "snacks at shelf (laundry); command says coffee table (*)",
        ),
        (
            "Follow Simone from the sink to the laundry",
            [find_person_with(name="simone"), has_skill("follow_person")],
            None,
        ),
        # ── Batch 11 ─────────────────────────────────────────────────────────
        (
            "Look for a waving person in the kitchen and tell the time",
            [navigates_to("kitchen"), find_person_with(gesture="waving"), has_skill("say")],
            None,
        ),
        (
            "Locate a pringles in the laundry then get it and give it to Adel in the bedroom",
            [has_skill("pick_up"), delivery_to_person("adel"), pick_up_followed_by_delivery(),
             correct_object_nav("pringles", world)],
            None,
        ),
        (
            "Give me a toothpaste from the cabinet",
            [has_skill("pick_up"), delivery_to_me()],
            "toothpaste NOT in known objects — planner may still attempt",
        ),
        # ── Batch 12 ─────────────────────────────────────────────────────────
        (
            "Go to the washing machine then look for a toy and take it and deliver it to Charlie in the living room",
            [has_skill("pick_up"), delivery_to_person("charlie"), pick_up_followed_by_delivery()],
            "rubiks_cube is only 'toy' but categorised as fabric in objects.yaml",
        ),
        (
            "Find a standing person in the living room and take them to the laundry table",
            [find_person_with(pose="standing"), has_skill("guide_person"), guide_person_to("laundry table")],
            None,
        ),
        (
            "Give me a cup from the tv stand",
            [has_skill("pick_up"), delivery_to_me(), correct_object_nav("cup", world)],
            "cup is at dishwasher (kitchen), not tv stand (*)",
        ),
        # ── Batch 13 ─────────────────────────────────────────────────────────
        (
            "Tell your teams name to the person pointing to the left in the kitchen",
            [navigates_to("kitchen"), has_skill("find_person"), has_skill("say")],
            None,
        ),
        (
            "Tell me what is the thinnest object on the bedside table",
            [has_skill("find_object_by_property"), ends_with("say"), navigates_to("bedside table")],
            None,
        ),
        (
            "Tell me what is the smallest toy on the cabinet",
            [has_skill("find_object_by_property"), ends_with("say")],
            "rubiks_cube is only 'toy' but categorised as fabric",
        ),
        # ── Batch 14 ─────────────────────────────────────────────────────────
        (
            "Say your teams country to the person raising their left arm in the living room",
            [navigates_to("living room"), has_skill("find_person"), has_skill("say")],
            None,
        ),
        (
            "Find a drink in the living room then grasp it and deliver it to the person pointing to the left in the laundry",
            [has_skill("pick_up"), has_skill("find_person"), has_skill("give_to_person"),
             pick_up_followed_by_delivery()],
            "drinks at cabinet (kitchen), not living room (*)",
        ),
        (
            "Navigate to the living room then look for a dishwasher tab and fetch it and bring it to the sitting person in the kitchen",
            [has_skill("pick_up"), has_skill("find_person"), has_skill("give_to_person"),
             pick_up_followed_by_delivery(), correct_object_nav("dishwasher_tab", world)],
            "dishwasher_tab at laundry table (laundry), not living room (*)",
        ),
        # ── Batch 15 ─────────────────────────────────────────────────────────
        (
            "Tell me what is the thinnest object on the tv stand",
            [has_skill("find_object_by_property"), ends_with("say"), navigates_to("tv stand")],
            None,
        ),
        (
            "Lead the person wearing an orange jacket from the entrance to the laundry table",
            [navigates_to("entrance"), find_person_with(clothes="orange jacket"),
             has_skill("guide_person"), guide_person_to("laundry table")],
            None,
        ),
        (
            "Follow the standing person at the washing machine",
            [navigates_to("washing machine"), find_person_with(pose="standing"), has_skill("follow_person")],
            None,
        ),
        # ── Batch 16 ─────────────────────────────────────────────────────────
        (
            "Find a person pointing to the left in the laundry and say your teams name",
            [navigates_to("laundry"), has_skill("find_person"), has_skill("say")],
            None,
        ),
        (
            "Look for a lying person in the living room and follow them",
            [navigates_to("living room"), find_person_with(pose="lying"), has_skill("follow_person")],
            None,
        ),
        (
            "Go to the laundry then look for a snack and grasp it and throw it in the trash",
            [navigates_to("laundry"), has_skill("pick_up"), place_at_trash(),
             pick_up_followed_by_delivery()],
            None,
        ),
        # ── Batch 17 ─────────────────────────────────────────────────────────
        (
            "Go to the tv stand then locate a drink and get it and bring it to Paris in the kitchen",
            [has_skill("pick_up"), delivery_to_person("paris"), pick_up_followed_by_delivery(),
             correct_object_nav("coke", world)],
            "drinks at cabinet (kitchen), not tv stand (*)",
        ),
        (
            "Find a snack in the bedroom then take it and bring it to me",
            [has_skill("pick_up"), delivery_to_me(), pick_up_followed_by_delivery(),
             correct_object_nav("pringles", world)],
            "snacks at shelf (laundry), not bedroom (*)",
        ),
        (
            "Tell me the gesture of the person in the kitchen",
            [navigates_to("kitchen"), has_skill("get_person_info"), ends_with("say")],
            None,
        ),
        # ── Batch 18 ─────────────────────────────────────────────────────────
        (
            "Bring me a toothpaste from the dishwasher",
            [has_skill("pick_up"), delivery_to_me()],
            "toothpaste NOT in known objects — planner may still attempt",
        ),
        (
            "Look for a pringles in the laundry then fetch it and put it on the bedside table",
            [has_skill("pick_up"), has_skill("place_object"), pick_up_followed_by_delivery(),
             navigates_to("bedside table"), correct_object_nav("pringles", world)],
            None,
        ),
        (
            "Find a person raising their right arm in the kitchen and tell the day of the month",
            [navigates_to("kitchen"), has_skill("find_person"), has_skill("say")],
            None,
        ),
        # ── Batch 19 ─────────────────────────────────────────────────────────
        (
            "Bring me a cup from the tv stand",
            [has_skill("pick_up"), delivery_to_me(), correct_object_nav("cup", world)],
            "cup at dishwasher (kitchen), not tv stand (*)",
        ),
        (
            "Tell me what is the smallest object on the coffee table",
            [has_skill("find_object_by_property"), ends_with("say"), navigates_to("coffee table")],
            None,
        ),
        (
            "Tell me the gesture of the person at the kitchen trash bin",
            [navigates_to("kitchen trash bin"), has_skill("get_person_info"), ends_with("say")],
            None,
        ),
        # ── Batch 20 ─────────────────────────────────────────────────────────
        (
            "Look for a fruit in the living room then take it and deliver it to me",
            [has_skill("pick_up"), delivery_to_me(), pick_up_followed_by_delivery(),
             correct_object_nav("apple", world)],
            "fruits at dinner table (kitchen), not living room (*)",
        ),
        (
            "Introduce yourself to the person wearing a yellow blouse in the kitchen and follow them",
            [navigates_to("kitchen"), find_person_with(clothes="yellow blouse"),
             has_skill("say"), has_skill("follow_person")],
            None,
        ),
        (
            "Guide the person wearing an orange coat from the tv stand to the entrance",
            [navigates_to("tv stand"), find_person_with(clothes="orange coat"),
             has_skill("guide_person"), guide_person_to("entrance")],
            None,
        ),
        # ── Batch 21 ─────────────────────────────────────────────────────────
        (
            "Navigate to the laundry then look for a plate and get it and deliver it to the waving person in the living room",
            [has_skill("pick_up"), has_skill("find_person"), has_skill("give_to_person"),
             pick_up_followed_by_delivery(), correct_object_nav("plate", world)],
            "plate at dishwasher (kitchen), not laundry (*)",
        ),
        (
            "Greet Angel in the bedroom and follow them to the bedside table",
            [navigates_to("bedroom"), find_person_with(name="angel"),
             has_skill("say"), has_skill("follow_person")],
            None,
        ),
        (
            "Introduce yourself to the person wearing a red jacket in the bedroom and follow them",
            [navigates_to("bedroom"), find_person_with(clothes="red jacket"),
             has_skill("say"), has_skill("follow_person")],
            None,
        ),
        # ── Batch 22 ─────────────────────────────────────────────────────────
        (
            "Navigate to the kitchen then look for the lying person and escort them to the dishwasher",
            [navigates_to("kitchen"), find_person_with(pose="lying"),
             has_skill("guide_person"), guide_person_to("dishwasher")],
            None,
        ),
        (
            "Grasp a grey shirt from the tv stand and give it to Paris in the bedroom",
            [has_skill("pick_up"), delivery_to_person("paris"), pick_up_followed_by_delivery(),
             correct_object_nav("grey_shirt", world)],
            "grey_shirt at laundry table (laundry), not tv stand (*)",
        ),
        (
            "Go to the bedroom then locate a bowl and fetch it and throw it in the trash",
            [has_skill("pick_up"), place_at_trash(), pick_up_followed_by_delivery(),
             correct_object_nav("bowl", world)],
            "bowl at dishwasher (kitchen), not bedroom (*)",
        ),
        # ── Batch 23 ─────────────────────────────────────────────────────────
        (
            "Meet Axel in the living room and tell what day is tomorrow",
            [navigates_to("living room"), find_person_with(name="axel"), has_skill("say")],
            None,
        ),
        (
            "Navigate to the washing machine then meet Simone and follow them",
            [navigates_to("washing machine"), find_person_with(name="simone"), has_skill("follow_person")],
            None,
        ),
        (
            "Grasp an apple from the tv stand and throw it in the trash",
            [has_skill("pick_up"), place_at_trash(), pick_up_followed_by_delivery(),
             correct_object_nav("apple", world)],
            "apple at dinner table (kitchen), not tv stand (*)",
        ),
        # ── Batch 24 ─────────────────────────────────────────────────────────
        (
            "Navigate to the tv stand then look for a toy and grasp it and give it to the standing person in the bedroom",
            [has_skill("pick_up"), has_skill("find_person"), has_skill("give_to_person"),
             pick_up_followed_by_delivery()],
            "rubiks_cube is only 'toy' but categorised as fabric",
        ),
        (
            "Meet Jane in the bedroom and follow them to the kitchen trash bin",
            [navigates_to("bedroom"), find_person_with(name="jane"), has_skill("follow_person")],
            None,
        ),
        (
            "Introduce yourself to Adel in the living room and guide them to the kitchen trash bin",
            [navigates_to("living room"), find_person_with(name="adel"),
             has_skill("say"), has_skill("guide_person"), guide_person_to("kitchen trash bin")],
            None,
        ),
        # ── Batch 25 ─────────────────────────────────────────────────────────
        (
            "Navigate to the refrigerator then meet Paris and lead them to the coffee table",
            [navigates_to("refrigerator"), find_person_with(name="paris"),
             has_skill("guide_person"), guide_person_to("coffee table")],
            None,
        ),
        (
            "Say your teams name to the person raising their left arm in the kitchen",
            [navigates_to("kitchen"), has_skill("find_person"), has_skill("say")],
            None,
        ),
        (
            "Grasp a pringles from the bedside table and deliver it to me",
            [has_skill("pick_up"), delivery_to_me(), pick_up_followed_by_delivery(),
             correct_object_nav("pringles", world)],
            "pringles at shelf (laundry), not bedside table (*)",
        ),
        # ── Batch 26 ─────────────────────────────────────────────────────────
        (
            "Locate a fabric in the bedroom then take it and throw it in the trash",
            [has_skill("pick_up"), place_at_trash(), pick_up_followed_by_delivery()],
            "fabrics at laundry table (laundry), not bedroom (*)",
        ),
        (
            "Tell me what is the thinnest object on the dishwasher",
            [has_skill("find_object_by_property"), ends_with("say"), navigates_to("dishwasher")],
            None,
        ),
        (
            "Follow Adel from the kitchen trash bin to the living room",
            [find_person_with(name="adel"), has_skill("follow_person")],
            None,
        ),
        # ── Batch 27 ─────────────────────────────────────────────────────────
        (
            "Navigate to the cabinet then look for the person raising their right arm and say what day is today",
            [navigates_to("cabinet"), has_skill("find_person"), has_skill("say")],
            None,
        ),
        (
            "Bring me a rubiks cube from the dishwasher",
            [has_skill("pick_up"), delivery_to_me(), correct_object_nav("rubiks_cube", world)],
            "rubiks_cube at bed (bedroom), not dishwasher (*)",
        ),
        (
            "Look for a knife in the kitchen then fetch it and bring it to Morgan in the bedroom",
            [has_skill("pick_up"), delivery_to_person("morgan"), pick_up_followed_by_delivery(),
             correct_object_nav("knife", world)],
            None,
        ),
        # ── Batch 28 ─────────────────────────────────────────────────────────
        (
            "Say your teams country to the person pointing to the right in the living room",
            [navigates_to("living room"), has_skill("find_person"), has_skill("say")],
            None,
        ),
        (
            "Tell your teams country to the person pointing to the left in the laundry",
            [navigates_to("laundry"), has_skill("find_person"), has_skill("say")],
            None,
        ),
        (
            "Give me a milk from the washing machine",
            [has_skill("pick_up"), delivery_to_me(), correct_object_nav("milk", world)],
            "milk at cabinet (kitchen), not washing machine (*)",
        ),
        # ── Batch 29 ─────────────────────────────────────────────────────────
        (
            "Say hello to Jules in the bedroom and tell something about yourself",
            [navigates_to("bedroom"), find_person_with(name="jules"), has_skill("say")],
            None,
        ),
        (
            "Go to the kitchen trash bin then find a spoon and fetch it and give it to Robin in the kitchen",
            [has_skill("pick_up"), delivery_to_person("robin"), pick_up_followed_by_delivery(),
             correct_object_nav("spoon", world)],
            None,
        ),
        (
            "Locate a dish in the living room then grasp it and throw it in the trash",
            [has_skill("pick_up"), place_at_trash(), pick_up_followed_by_delivery()],
            "dishes at dishwasher (kitchen), not living room (*)",
        ),
        # ── Batch 30 ─────────────────────────────────────────────────────────
        (
            "Go to the washing machine then find a spoon and grasp it and put it on the tv stand",
            [has_skill("pick_up"), ends_with("place_object"), pick_up_followed_by_delivery(),
             correct_object_nav("spoon", world)],
            "spoon at dishwasher (kitchen), not washing machine (*)",
        ),
        (
            "Tell me what is the smallest object on the cabinet",
            [has_skill("find_object_by_property"), ends_with("say"), navigates_to("cabinet")],
            None,
        ),
        (
            "Escort Adel from the refrigerator to the kitchen",
            [navigates_to("refrigerator"), find_person_with(name="adel"),
             has_skill("guide_person"), guide_person_to("kitchen")],
            None,
        ),
        # ── Batch 31 ─────────────────────────────────────────────────────────
        (
            "Give me a coke from the laundry trash bin",
            [has_skill("pick_up"), delivery_to_me(), correct_object_nav("coke", world)],
            "coke at cabinet (kitchen), not laundry trash bin (*)",
        ),
        (
            "Guide the lying person from the sink to the kitchen trash bin",
            [navigates_to("sink"), find_person_with(pose="lying"),
             has_skill("guide_person"), guide_person_to("kitchen trash bin")],
            None,
        ),
        (
            "Look for a black shirt in the bedroom then get it and deliver it to me",
            [has_skill("pick_up"), delivery_to_me(), pick_up_followed_by_delivery(),
             correct_object_nav("black_shirt", world)],
            "black_shirt at laundry table (laundry), not bedroom (*)",
        ),
        # ── Batch 32 ─────────────────────────────────────────────────────────
        (
            "Find a snack in the bedroom then fetch it and throw it in the trash",
            [has_skill("pick_up"), place_at_trash(), pick_up_followed_by_delivery()],
            "snacks at shelf (laundry), not bedroom (*)",
        ),
        (
            "Go to the living room then find the person pointing to the left and tell your teams name",
            [navigates_to("living room"), has_skill("find_person"), has_skill("say")],
            None,
        ),
        (
            "Tell me what is the biggest object on the dishwasher",
            [has_skill("find_object_by_property"), ends_with("say"), navigates_to("dishwasher")],
            None,
        ),
        # ── Batch 33 ─────────────────────────────────────────────────────────
        (
            "Tell me what is the thinnest dish on the washing machine",
            [has_skill("find_object_by_property"), ends_with("say")],
            None,
        ),
        (
            "Tell me what is the largest object on the bedside table",
            [has_skill("find_object_by_property"), ends_with("say"), navigates_to("bedside table")],
            None,
        ),
        (
            "Navigate to the entrance then meet Charlie and say what day is today",
            [navigates_to("entrance"), find_person_with(name="charlie"), has_skill("say")],
            None,
        ),
        # ── Batch 34 ─────────────────────────────────────────────────────────
        (
            "Bring me a dishwasher tab from the coffee table",
            [has_skill("pick_up"), delivery_to_me(), correct_object_nav("dishwasher_tab", world)],
            "dishwasher_tab at laundry table (laundry), not coffee table (*)",
        ),
        (
            "Meet Simone in the living room and follow them to the tv stand",
            [navigates_to("living room"), find_person_with(name="simone"), has_skill("follow_person")],
            None,
        ),
        (
            "Get a rubiks cube from the cabinet and throw it in the trash",
            [has_skill("pick_up"), place_at_trash(), pick_up_followed_by_delivery(),
             correct_object_nav("rubiks_cube", world)],
            "rubiks_cube at bed (bedroom), not cabinet (*)",
        ),
        # ── Batch 35 ─────────────────────────────────────────────────────────
        (
            "Navigate to the refrigerator then meet Adel and follow them to the bedroom",
            [navigates_to("refrigerator"), find_person_with(name="adel"), has_skill("follow_person")],
            None,
        ),
        (
            "Tell me what is the biggest object on the laundry trash bin",
            [has_skill("find_object_by_property"), ends_with("say"), navigates_to("laundry trash bin")],
            None,
        ),
        (
            "Tell me how many snacks there are on the dishwasher",
            [has_skill("count_objects"), ends_with("say")],
            "snacks at shelf (laundry); count_objects should still handle (*)",
        ),
        # ── Batch 36 ─────────────────────────────────────────────────────────
        (
            "Tell me what is the largest object on the dishwasher",
            [has_skill("find_object_by_property"), ends_with("say"), navigates_to("dishwasher")],
            None,
        ),
        (
            "Go to the tv stand then find the sitting person and say what day is tomorrow",
            [navigates_to("tv stand"), find_person_with(pose="sitting"), has_skill("say")],
            None,
        ),
        (
            "Salute Morgan in the kitchen and follow them to the laundry",
            [navigates_to("kitchen"), find_person_with(name="morgan"),
             has_skill("say"), has_skill("follow_person")],
            None,
        ),
    ]


# ── Output helpers ────────────────────────────────────────────────────────────
USE_COLOR = True

def _c(code, text):
    return f"{code}{text}\033[0m" if USE_COLOR else text

def green(t):  return _c("\033[92m", t)
def red(t):    return _c("\033[91m", t)
def yellow(t): return _c("\033[93m", t)
def cyan(t):   return _c("\033[96m", t)
def bold(t):   return _c("\033[1m",  t)
def dim(t):    return _c("\033[2m",  t)


def print_plan(plan: dict):
    steps   = plan.get("steps", [])
    original = plan.get("_original_command", "")
    cleaned  = plan.get("_cleaned_command", "")
    if original and cleaned and original != cleaned:
        print(dim(f"  Cleaned: '{original}' → '{cleaned}'"))
    print(dim(f"  Plan: {plan.get('plan_description', '')}"))
    for i, step in enumerate(steps, 1):
        skill = step.get("skill", "?")
        args  = step.get("args", {})
        args_str = ", ".join(f"{k}={v!r}" for k, v in args.items()) if args else ""
        print(dim(f"  {i:2d}. {skill}({args_str})"))


def run_checks(checks: List[Check], steps: list) -> tuple:
    results = []
    for check in checks:
        try:
            passed = check.fn(steps)
        except Exception as e:
            passed = False
            check = Check(check.label + f" [ERR: {e}]", check.fn, check.severity)
        results.append((check, passed))
    return results


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    global USE_COLOR

    parser = argparse.ArgumentParser(description="Standalone GPSR planner test")
    parser.add_argument("--model",    default="gemma3",              help="Ollama model name")
    parser.add_argument("--host",     default="http://localhost:11434", help="Ollama host")
    parser.add_argument("--batch",    type=int, default=None,        help="Only run one batch (0-indexed)")
    parser.add_argument("--cmd",      default=None,                  help="Run a single ad-hoc command")
    parser.add_argument("--no-color", action="store_true",           help="Disable colour output")
    parser.add_argument("--json-out", default=None,                  help="Write full results to JSON file")
    args = parser.parse_args()

    if args.no_color:
        USE_COLOR = False

    print(bold(f"\n{'═'*70}"))
    print(bold(f"  GPSR Planner Test  |  model: {args.model}"))
    print(bold(f"{'═'*70}\n"))

    # Load world
    try:
        world = load_world()
    except Exception as e:
        print(red(f"Failed to load world config: {e}"))
        sys.exit(1)
    print(dim(f"  Locations: {world['locations_str'][:80]}…"))
    print()

    # Build backend
    backend = LocalBackend(model=args.model, host=args.host)

    # Single ad-hoc command
    if args.cmd:
        print(bold(f"Command: {args.cmd}"))
        t0 = time.perf_counter()
        plan = run_pipeline(backend, world, args.cmd)
        print(f"  [{time.perf_counter()-t0:.1f}s]")
        print_plan(plan)
        return

    # Build test cases
    test_cases = build_test_cases(world)

    # Optionally filter to a single batch (groups of 3)
    if args.batch is not None:
        lo = args.batch * 3
        hi = lo + 3
        test_cases = test_cases[lo:hi]

    total = len(test_cases)
    passed_cases = 0
    failed_cases = 0
    all_results  = []

    for idx, (cmd, checks, note) in enumerate(test_cases):
        batch_num = idx // 3 + 1
        cmd_num   = idx % 3 + 1
        header    = f"[Batch {batch_num:02d} #{cmd_num}]"

        print(bold(f"{header} {cmd}"))
        if note:
            print(yellow(f"  NOTE: {note}"))

        t0 = time.perf_counter()
        try:
            plan = run_pipeline(backend, world, cmd)
        except Exception as e:
            print(red(f"  PIPELINE ERROR: {e}"))
            failed_cases += 1
            all_results.append({"cmd": cmd, "error": str(e)})
            print()
            continue

        elapsed = time.perf_counter() - t0
        steps   = plan.get("steps", [])
        print_plan(plan)

        check_results = run_checks(checks, steps)
        case_ok = True
        for check, passed in check_results:
            if passed:
                print(f"  {green('✓')} {check.label}")
            else:
                if check.severity == "WARN":
                    print(f"  {yellow('?')} {check.label}")
                else:
                    print(f"  {red('✗')} {check.label}")
                    case_ok = False

        print(dim(f"  [{elapsed:.1f}s]"))

        if case_ok:
            passed_cases += 1
            print(green(f"  → PASS"))
        else:
            failed_cases += 1
            print(red(f"  → FAIL"))

        all_results.append({
            "cmd":   cmd,
            "plan":  plan,
            "checks": [
                {"label": c.label, "passed": p, "severity": c.severity}
                for c, p in check_results
            ],
            "case_pass": case_ok,
            "elapsed_sec": elapsed,
        })
        print()

    # Summary
    print(bold(f"\n{'═'*70}"))
    print(bold(f"  RESULTS: {passed_cases}/{total} passed  |  {failed_cases} failed"))
    print(bold(f"{'═'*70}\n"))

    if args.json_out:
        with open(args.json_out, "w") as f:
            json.dump(all_results, f, indent=2, default=str)
        print(dim(f"Full results written to {args.json_out}\n"))

    sys.exit(0 if failed_cases == 0 else 1)


if __name__ == "__main__":
    main()
