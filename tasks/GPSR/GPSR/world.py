"""Load robot world config from yaml and format it for the LLM planner."""

import os
from pathlib import Path

import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - non-ROS/macOS fallback
    get_package_share_directory = None

__all__ = [
    "build_world",
    "compact_skill_lines",
    "format_objects",
    "format_people",
    "load_locations",
    "normalise_name",
    "placeable_locations",
    "room_names",
    "selected_skill_lines",
    "sublocation_rooms",
    "vocabulary_prompt",
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


def normalise_name(name) -> str:
    """Lower-case and replace underscores with spaces ("Red_Bull" -> "red bull")."""
    return str(name or "").strip().lower().replace("_", " ")


def _point_in_polygon(x: float, y: float, polygon) -> bool:
    """Ray-casting point-in-polygon test; polygon is a list of [x, y] vertices."""
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            if x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
                inside = not inside
    return inside


def _polygon_vertices(polygon) -> list:
    """locations.yaml polygons are {top_left: [x, y], ...} (in order) or a list of [x, y]."""
    if isinstance(polygon, dict):
        return [list(v) for v in polygon.values()]
    return [list(v) for v in polygon or []]


def room_names(locations: dict) -> list:
    """Locations that are rooms, i.e. have a `polygon` in locations.yaml."""
    return [name for name, info in locations.items() if (info or {}).get("polygon")]


def sublocation_rooms(locations: dict) -> dict:
    """Map each non-room location to the room it is in: {sub-location: room}.

    Uses the location's explicit `room:` key if set, otherwise the room whose polygon
    contains the location's position. Locations in no room are left out.
    """
    rooms = room_names(locations)
    result = {}
    for name, info in locations.items():
        info = info or {}
        if name in rooms:
            continue
        room = info.get("room")
        if not room:
            pos = info.get("position") or {}
            if "x" in pos and "y" in pos:
                for candidate in rooms:
                    vertices = _polygon_vertices(locations[candidate].get("polygon"))
                    if len(vertices) >= 3 and _point_in_polygon(
                        float(pos["x"]), float(pos["y"]), vertices
                    ):
                        room = candidate
                        break
        if room:
            result[name] = room
    return result


def format_objects(objects: dict, sublocation_room: dict = None) -> str:
    """Format the objects dict as a single line for the planner prompt."""
    sublocation_room = sublocation_room or {}
    parts = []
    for name, obj in objects.items():
        obj = obj or {}
        subloc = obj.get("location", "?")
        room = sublocation_room.get(subloc, "?")
        parts.append(
            f"{normalise_name(name)} ({obj.get('category', '?')} at {subloc} in {room})"
        )
    return ", ".join(parts) or "none"


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
    if get_package_share_directory is not None:
        try:
            return os.path.join(get_package_share_directory("GPSR"), "config", filename)
        except Exception:
            pass
    return str(Path(__file__).resolve().parents[1] / "config" / filename)


def load_skills_text(node):
    """Load skills.yaml as plain text (comments stripped)."""
    path = _pkg_config(node, "skills.yaml")
    if not os.path.exists(path):
        return ""
    with open(path) as f:
        lines = [l.rstrip() for l in f if l.strip() and not l.startswith("#")]
    return "\n".join(lines)


def load_locations(node):
    """Load locations.yaml → {name: {position, orientation, placeable, ...}}."""
    path = _pkg_config(node, "locations.yaml")
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return data.get("locations", {})


def placeable_locations(locations: dict) -> list:
    """Return names of locations where place_object is allowed (placeable: true)."""
    return [name for name, info in locations.items() if info.get("placeable", False)]


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
    """Load general_knowledge.yaml and append live date/time."""
    import datetime

    path = _pkg_config(node, "general_knowledge.yaml")
    if not os.path.exists(path):
        base = ""
    else:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        base = data.get("info", "")

    now = datetime.datetime.now()
    date_str = now.strftime("%-d %B %Y")
    day_of_week = now.strftime("%A")
    day_of_month = now.strftime("%-d")
    time_str = now.strftime("%H:%M")
    dynamic = (
        f" The current date is {date_str}."
        f" The day of the week is {day_of_week}."
        f" The day of the month is {day_of_month}."
        f" The current time is {time_str}."
        f" Tomorrow is {(now + datetime.timedelta(days=1)).strftime('%A')}."
    )
    return base + dynamic


def build_world(node) -> dict:
    """Load all config yaml and return the world dict passed to the planner."""
    skills_text = load_skills_text(node)
    locations = load_locations(node)
    return {
        "locations": locations,
        "placement_locations": placeable_locations(locations),
        "sublocation_rooms": sublocation_rooms(locations),
        "objects": load_objects(node),
        "people": load_people(node),
        "general_knowledge": load_general_knowledge(node),
        "skill_lines": compact_skill_lines(skills_text),
    }


def vocabulary_prompt(node=None) -> str:
    """Whisper `initial_prompt` built from the people, objects and locations in config.

    Whisper treats the prompt as the text spoken just before the audio, so it is
    written as an example robot command followed by the vocabulary, which biases
    recognition towards the words declared for the current arena (a bare word list
    tends to make Whisper output lists).
    """
    locations = load_locations(node)
    objects = load_objects(node)
    rooms = room_names(locations)
    places = [n for n in locations if n not in rooms]
    object_words = []
    for name, info in objects.items():
        object_words.append(normalise_name(name))
        object_words.extend(normalise_name(a) for a in (info or {}).get("aliases") or [])
    categories = []
    for info in objects.values():
        category = normalise_name((info or {}).get("category"))
        if category and category not in categories:
            categories.append(category)

    def unique(words):
        seen, out = set(), []
        for word in words:
            if word and word.lower() not in seen:
                seen.add(word.lower())
                out.append(word)
        return ", ".join(out)

    example_place = next((p for p in places if locations[p].get("placeable")), "table")
    example_object = normalise_name(next(iter(objects), "object"))
    return (
        f"Robot, go to the {example_place}, find the {example_object} and bring it to me. "
        f"People: {unique(str(p).capitalize() for p in load_people(node))}. "
        f"Rooms: {unique(rooms)}. "
        f"Places: {unique(places)}. "
        f"Objects: {unique(object_words)}. "
        f"Categories: {unique(categories)}."
    )
