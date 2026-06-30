"""Load robot world config from yaml and format it for the LLM planner."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory

__all__ = [
    "build_world",
    "compact_skill_lines",
    "format_objects",
    "format_people",
    "load_locations",
    "selected_skill_lines",
]


def compact_skill_lines(skills_text: str) -> str:
    """Turn raw skills.yaml text into compact lines for LLM prompts (name, args, purpose)."""
    lines = []
    for raw in skills_text.splitlines():
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
        lines.append(f"{name}({args}) — {purpose}")
    return "\n".join(lines)


def format_objects(objects: dict) -> str:
    """Format the objects dict as a single line for the planner prompt."""
    return (
        ", ".join(
            f"{name} ({obj.get('category', '?')} in {obj.get('location', '?')})"
            for name, obj in objects.items()
        )
        or "none"
    )


def format_people(people: dict) -> str:
    """Format the people dict as a single line for the planner prompt."""
    return (
        ", ".join(
            f"{name} ({info.get('gender', '?')})" for name, info in people.items()
        )
        or "none"
    )


def selected_skill_lines(selected_skills: list, all_skill_lines: str) -> str:
    """Return only the skill lines chosen by the skill selector."""
    if not selected_skills:
        return "say(text) — speak aloud"
    result = []
    for line in all_skill_lines.splitlines():
        skill_name = line.split("(")[0].strip()
        if skill_name in selected_skills:
            result.append(line)
    return "\n".join(result) if result else all_skill_lines


def _pkg_config(node, filename):
    """Absolute path to a file under share/GPSR/config/."""
    return os.path.join(get_package_share_directory("GPSR"), "config", filename)


def load_skills_text(node):
    """Load skills.yaml as plain text (comments stripped)."""
    path = _pkg_config(node, "skills.yaml")
    if not os.path.exists(path):
        return ""
    with open(path) as f:
        lines = [l.rstrip() for l in f if l.strip() and not l.startswith("#")]
    return "\n".join(lines)


def load_locations(node):
    """Load locations.yaml → {name: {position, orientation}}."""
    path = _pkg_config(node, "locations.yaml")
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return data.get("locations", {})


def load_objects(node):
    """Load objects.yaml → {name: {category, location, ...}}."""
    path = _pkg_config(node, "objects.yaml")
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return data.get("objects", {})


def load_people(node):
    """Load people.yaml → {name: {gender, ...}}."""
    path = _pkg_config(node, "people.yaml")
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return data.get("people", {})


def load_general_knowledge(node):
    """Load general_knowledge.yaml → free-text string for the planner."""
    path = _pkg_config(node, "general_knowledge.yaml")
    if not os.path.exists(path):
        return ""
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return data.get("info", "")


def build_world(node) -> dict:
    """Load all config yaml and return the world dict passed to the planner."""
    skills_text = load_skills_text(node)
    return {
        "locations": load_locations(node),
        "objects": load_objects(node),
        "people": load_people(node),
        "general_knowledge": load_general_knowledge(node),
        "skill_lines": compact_skill_lines(skills_text),
    }
