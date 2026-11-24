"""Deterministic sanity checks around the LLM planner.

The local LLM plans almost anything it is given, including garbled speech
transcriptions. These checks reject a command before planning if it does not look
like a robot command about things the robot knows, and reject a plan afterwards if it
refers to locations, objects or skills that are not in the config.
"""

import re

from GPSR.world import normalise_name

# Words a command starts with / contains (see TRANSCRIPTION_CLEANER_PROMPT)
COMMAND_WORDS = {
    "go",
    "navigate",
    "find",
    "locate",
    "bring",
    "get",
    "fetch",
    "take",
    "put",
    "place",
    "deliver",
    "count",
    "tell",
    "say",
    "follow",
    "escort",
    "guide",
    "lead",
    "answer",
    "introduce",
    "greet",
    "meet",
    "give",
    "pick",
    "carry",
    "move",
    "look",
    "search",
    "come",
    "return",
    "describe",
    "salute",
    "what",
    "how",
    "where",
    "who",
    "which",
    "whats",
}

# Things a command may be about that are not in the config files
GENERIC_ENTITIES = {
    "me",
    "you",
    "your",
    "yourself",
    "person",
    "people",
    "someone",
    "somebody",
    "anyone",
    "everyone",
    "operator",
    "question",
    "questions",
    "team",
    "name",
    "time",
    "date",
    "day",
    "week",
    "month",
    "tomorrow",
    "today",
    "joke",
    "object",
    "objects",
    "something",
    "thing",
    "things",
    "waving",
    "sitting",
    "standing",
    "lying",
    "pointing",
    "raising",
    "wearing",
    "shirt",
    "gesture",
    "pose",
}

LOCATION_ARGS = ("location", "start", "end", "destination", "target", "surface")
OBJECT_ARGS = ("object",)
GENERIC_OBJECT_ARGS = {
    "",
    "object",
    "objects",
    "it",
    "item",
    "items",
    "something",
    "thing",
}


def _plural_forms(word: str) -> set:
    forms = {word, word + "s"}
    if word.endswith("y"):
        forms.add(word[:-1] + "ies")
    if word.endswith(("s", "sh", "ch", "x")):
        forms.add(word + "es")
    return forms


def _text(value) -> str:
    """Lower-case, `_` -> space, punctuation removed, single spaces."""
    text = normalise_name(value)
    text = re.sub(r"[^a-z0-9 ]+", " ", text)
    return " ".join(text.split())


def _known_objects(world: dict) -> set:
    """Object names, aliases and categories (singular and plural)."""
    known = set()
    for name, info in (world.get("objects") or {}).items():
        info = info or {}
        known.add(_text(name))
        known.update(_text(a) for a in info.get("aliases") or [])
        if info.get("category"):
            known.update(_plural_forms(_text(info["category"])))
    known.discard("")
    return known


def _known_locations(world: dict) -> set:
    return {_text(name) for name in (world.get("locations") or {})}


def _contains_phrase(text: str, phrase: str) -> bool:
    return re.search(rf"\b{re.escape(phrase)}\b", text) is not None


def check_command(command: str, world: dict) -> tuple[bool, str]:
    """Whether `command` looks like a robot command about known things.

    Requires a command verb / question word AND a mention of something the robot
    knows: a location, object, object category, person, or a generic entity such as
    "me" or "person". Returns (ok, reason).
    """
    text = _text(command)
    words = set(text.split())
    if not words & COMMAND_WORDS:
        return False, f"no command verb in '{command}'"

    entities = (
        _known_locations(world)
        | _known_objects(world)
        | {_text(p) for p in (world.get("people") or {})}
    )
    if words & GENERIC_ENTITIES:
        return True, ""
    if any(_contains_phrase(text, e) for e in entities):
        return True, ""
    return False, f"'{command}' mentions no known location, object or person"


def validate_plan(steps: list, world: dict) -> list:
    """Problems with a planned list of steps (empty list = plan is OK)."""
    problems = []
    known_skills = {
        line.split("(")[0].strip()
        for line in (world.get("skill_lines") or "").splitlines()
        if line.strip()
    }
    locations = _known_locations(world)
    objects = _known_objects(world)

    for i, step in enumerate(steps, 1):
        skill = step.get("skill", "")
        args = step.get("args") or {}
        if known_skills and skill not in known_skills:
            problems.append(f"step {i}: unknown skill '{skill}'")
        for key in LOCATION_ARGS:
            value = args.get(key)
            if (
                isinstance(value, str)
                and value.strip()
                and _text(value) not in locations
            ):
                problems.append(f"step {i} {skill}: unknown location '{value}'")
        for key in OBJECT_ARGS:
            value = args.get(key)
            if (
                isinstance(value, str)
                and _text(value) not in GENERIC_OBJECT_ARGS
                and _text(value) not in objects
            ):
                problems.append(f"step {i} {skill}: unknown object '{value}'")
    return problems


YES_WORDS = {"yes", "yeah", "yep", "yup", "correct", "right", "affirmative", "sure"}
NO_WORDS = {"no", "nope", "not", "wrong", "incorrect", "negative"}


def is_yes(answer) -> bool:
    """True only for a clear yes; no, unclear or empty answers are False."""
    words = set(_text(answer).split())
    return bool(words & YES_WORDS) and not words & NO_WORDS
