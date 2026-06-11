import os

import yaml
from ament_index_python.packages import get_package_share_directory


def _pkg_config(node, filename):
    package = node.get_parameter("locations_package").value
    return os.path.join(get_package_share_directory(package), "config", filename)


def load_skills_text(node):
    path = _pkg_config(node, "skills.yaml")
    if not os.path.exists(path):
        return ""
    with open(path) as f:
        lines = [l.rstrip() for l in f if l.strip() and not l.startswith("#")]
    return "\n".join(lines)


def compact_skill_lines(skills_text):
    """'name: Long desc. Args: a (string), b (string).' → 'name(a, b) — purpose'"""
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


def load_locations(node):
    package = node.get_parameter("locations_package").value
    locations_file = node.get_parameter("locations_file").value
    path = os.path.join(get_package_share_directory(package), locations_file)
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return data.get("locations", {})


def load_objects(node):
    path = _pkg_config(node, "objects.yaml")
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return data.get("objects", {})


def load_people(node):
    path = _pkg_config(node, "people.yaml")
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return data.get("people", {})


def load_general_knowledge(node):
    path = _pkg_config(node, "general_knowledge.yaml")
    if not os.path.exists(path):
        return ""
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return data.get("info", "")


def format_objects(objects):
    return ", ".join(
        f"{name} ({obj.get('category','?')} in {obj.get('location','?')})"
        for name, obj in objects.items()
    ) or "none"


def format_people(people):
    return ", ".join(
        f"{name} ({info.get('gender','?')})" for name, info in people.items()
    ) or "none"


def selected_skill_lines(selected_skills: list, all_skill_lines: str) -> str:
    if not selected_skills:
        return "say(text) — speak aloud"
    result = []
    for line in all_skill_lines.splitlines():
        skill_name = line.split("(")[0].strip()
        if skill_name in selected_skills:
            result.append(line)
    return "\n".join(result) if result else all_skill_lines
