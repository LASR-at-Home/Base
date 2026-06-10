import json
import os
import time

import yasmin
import yaml
from ament_index_python.packages import get_package_share_directory

from GPSR.agent import Agent

# --- Stage 1: single scene analyst (JSON world report) ---

SCENE_ANALYST_PROMPT = """You are the scene analyst for a robot. Classify entities from the command.
Output ONLY one JSON object. Each category uses ONLY its own YAML file — never cross-check.

=== locations.yaml ONLY (for locations.known / locations.unknown) ===
Room names: {locations}

=== objects.yaml ONLY (for objects.known / objects.unknown) ===
Object names: {object_names}
Details (metadata only — do NOT use "in kitchen" to classify locations):
{objects}

=== people.yaml ONLY (for people.known / people.unknown) ===
People names: {people_names}
Details: {people}

CHAIN OF THOUGHT — fill "reasoning" first:
1. MENTIONED: extract places → locations.mentioned; grabbable things → objects.mentioned; persons → people.mentioned.
2. KNOWN per category: locations → check locations.yaml ONLY; objects → objects.yaml ONLY; people → people.yaml ONLY.
3. PARTITION: each mentioned item in "known" OR "unknown" for its category.
4. TASK: requires_scene, task_type, has_find_intent.

Output schema (always include every field):
{{
  "reasoning": "1.MENTIONED: ... 2.KNOWN CHECK: ... 3.PARTITION: ... 4.TASK: ...",
  "requires_scene": true,
  "locations": {{"mentioned": [], "known": [], "unknown": []}},
  "people": {{"mentioned": [], "known": [], "unknown": []}},
  "objects": {{"mentioned": [], "known": [], "unknown": []}},
  "task_type": "search|interaction|navigation|manipulation|question",
  "has_find_intent": false,
  "required_skills": []
}}

HARD RULES:
- locations.known ⊆ locations.yaml keys only. waste basket, sofa, refrigerator, coatrack, bathroom → locations.unknown (not in yaml).
- objects.known ⊆ objects.yaml keys only. mustard, apple, cola → objects.known. refrigerator is NOT an object.
- people.known ⊆ people.yaml keys only. simone, charlie → people.unknown; john, emma, robin → people.known.
- Never copy a room name from objects.yaml "location" field into locations.known unless the command mentions that room.
- Do NOT add to "mentioned" anything not in the command.
- requires_scene=false ONLY for team/robot questions with zero places/people/objects.

has_find_intent:
- true: find, locate, meet, search, tell me what (in the environment)
- false: direct lead/escort/guide without meet/find; navigation only; team questions

EXAMPLES:

Command: what is your affiliation
JSON: {{"reasoning": "1.MENTIONED: none. 2.KNOWN CHECK: n/a. 3.PARTITION: empty. 4.TASK: question, requires_scene=false.", "requires_scene": false, "locations": {{"mentioned": [], "known": [], "unknown": []}}, "people": {{"mentioned": [], "known": [], "unknown": []}}, "objects": {{"mentioned": [], "known": [], "unknown": []}}, "task_type": "question", "has_find_intent": false, "required_skills": ["say"]}}

Command: lead Simone from the coatrack to the bathroom
JSON: {{"reasoning": "1.MENTIONED: coatrack, bathroom, simone. 2.KNOWN CHECK: none in allowlists. 3.PARTITION: all unknown. 4.TASK: interaction, lead without find.", "requires_scene": true, "locations": {{"mentioned": ["coatrack", "bathroom"], "known": [], "unknown": ["coatrack", "bathroom"]}}, "people": {{"mentioned": ["simone"], "known": [], "unknown": ["simone"]}}, "objects": {{"mentioned": [], "known": [], "unknown": []}}, "task_type": "interaction", "has_find_intent": false, "required_skills": ["guide_person"]}}

Command: meet Charlie in the kitchen and escort them to the bedroom
JSON: {{"reasoning": "1.MENTIONED: kitchen, bedroom, charlie. 2.KNOWN CHECK: kitchen+bedroom in locations; charlie NOT in people. 3.PARTITION: loc known, charlie unknown. 4.TASK: meet=true.", "requires_scene": true, "locations": {{"mentioned": ["kitchen", "bedroom"], "known": ["kitchen", "bedroom"], "unknown": []}}, "people": {{"mentioned": ["charlie"], "known": [], "unknown": ["charlie"]}}, "objects": {{"mentioned": [], "known": [], "unknown": []}}, "task_type": "interaction", "has_find_intent": true, "required_skills": ["go_to_location", "find_person", "guide_person"]}}

Command: meet Emma in the living room and escort her to the kitchen
JSON: {{"reasoning": "1.MENTIONED: living room, kitchen, emma. 2.KNOWN CHECK: all in allowlists. 3.PARTITION: all known. 4.TASK: meet=true.", "requires_scene": true, "locations": {{"mentioned": ["living room", "kitchen"], "known": ["living room", "kitchen"], "unknown": []}}, "people": {{"mentioned": ["emma"], "known": ["emma"], "unknown": []}}, "objects": {{"mentioned": [], "known": [], "unknown": []}}, "task_type": "interaction", "has_find_intent": true, "required_skills": ["go_to_location", "find_person", "guide_person"]}}

Command: go to the bedroom find the mustard take it and bring it to the kitchen
JSON: {{"reasoning": "1.MENTIONED: bedroom, kitchen, mustard. 2.KNOWN CHECK: all in allowlists. 3.PARTITION: all known. 4.TASK: manipulation, find=true.", "requires_scene": true, "locations": {{"mentioned": ["bedroom", "kitchen"], "known": ["bedroom", "kitchen"], "unknown": []}}, "people": {{"mentioned": [], "known": [], "unknown": []}}, "objects": {{"mentioned": ["mustard"], "known": ["mustard"], "unknown": []}}, "task_type": "manipulation", "has_find_intent": true, "required_skills": ["go_to_location", "find_object", "pick_up", "place_object"]}}

Command: go to the waste basket then find a mustard and take it and put it on the refrigerator
JSON: {{"reasoning": "1.MENTIONED: waste basket, refrigerator, mustard. 2.KNOWN CHECK: mustard=object; waste basket+refrigerator are surfaces → locations not objects. 3.PARTITION: loc unknown, mustard known. 4.TASK: manipulation.", "requires_scene": true, "locations": {{"mentioned": ["waste basket", "refrigerator"], "known": [], "unknown": ["waste basket", "refrigerator"]}}, "people": {{"mentioned": [], "known": [], "unknown": []}}, "objects": {{"mentioned": ["mustard"], "known": ["mustard"], "unknown": []}}, "task_type": "manipulation", "has_find_intent": true, "required_skills": ["go_to_location", "find_object", "pick_up", "place_object"]}}

Command: tell me what is the biggest object on the sofa
JSON: {{"reasoning": "1.MENTIONED: sofa only. 2.KNOWN CHECK: sofa NOT in locations. 3.PARTITION: sofa unknown. 4.TASK: search.", "requires_scene": true, "locations": {{"mentioned": ["sofa"], "known": [], "unknown": ["sofa"]}}, "people": {{"mentioned": [], "known": [], "unknown": []}}, "objects": {{"mentioned": [], "known": [], "unknown": []}}, "task_type": "search", "has_find_intent": true, "required_skills": ["find_object_by_property"]}}

Command: find the apple in the kitchen
JSON: {{"reasoning": "1.MENTIONED: kitchen, apple. 2.KNOWN CHECK: both in allowlists. 3.PARTITION: both known. 4.TASK: search.", "requires_scene": true, "locations": {{"mentioned": ["kitchen"], "known": ["kitchen"], "unknown": []}}, "people": {{"mentioned": [], "known": [], "unknown": []}}, "objects": {{"mentioned": ["apple"], "known": ["apple"], "unknown": []}}, "task_type": "search", "has_find_intent": true, "required_skills": ["go_to_location", "find_object"]}}

Command: locate a snack in the bathroom then take it and put it on the sofa
JSON: {{"reasoning": "1.MENTIONED: bathroom, sofa, snack. 2.KNOWN CHECK: none in allowlists. 3.PARTITION: all unknown. 4.TASK: manipulation.", "requires_scene": true, "locations": {{"mentioned": ["bathroom", "sofa"], "known": [], "unknown": ["bathroom", "sofa"]}}, "people": {{"mentioned": [], "known": [], "unknown": []}}, "objects": {{"mentioned": ["snack"], "known": [], "unknown": ["snack"]}}, "task_type": "manipulation", "has_find_intent": true, "required_skills": ["go_to_location", "find_object", "pick_up", "place_object"]}}

Command: guide John from the kitchen to the living room
JSON: {{"reasoning": "1.MENTIONED: kitchen, living room, john. 2.KNOWN CHECK: all in allowlists. 3.PARTITION: all known. 4.TASK: guide, no find.", "requires_scene": true, "locations": {{"mentioned": ["kitchen", "living room"], "known": ["kitchen", "living room"], "unknown": []}}, "people": {{"mentioned": ["john"], "known": ["john"], "unknown": []}}, "objects": {{"mentioned": [], "known": [], "unknown": []}}, "task_type": "interaction", "has_find_intent": false, "required_skills": ["guide_person"]}}

Command: go to the kitchen
JSON: {{"reasoning": "1.MENTIONED: kitchen. 2.KNOWN CHECK: kitchen in locations. 3.PARTITION: known. 4.TASK: navigation.", "requires_scene": true, "locations": {{"mentioned": ["kitchen"], "known": ["kitchen"], "unknown": []}}, "people": {{"mentioned": [], "known": [], "unknown": []}}, "objects": {{"mentioned": [], "known": [], "unknown": []}}, "task_type": "navigation", "has_find_intent": false, "required_skills": ["go_to_location"]}}

Command: {command}
JSON: """

# --- Stage 2b: lightweight planner for general questions (requires_scene=false) ---

GENERAL_PLANNER_PROMPT = """You are the robot from team {team_name} ({team_affiliation}, {team_country}).
Answer general questions about the team or robot with a single say step. No scene reasoning needed.

TEAM INFO:
- name: {team_name}
- affiliation: {team_affiliation}
- country: {team_country}

Output ONLY JSON: {{"plan_description": "...", "steps": [{{"skill": "say", "args": {{"text": "..."}}}}]}}
The "text" is only the spoken answer — concise and factual.

EXAMPLES:

Command: what is your affiliation
Scene: {{"requires_scene": false, "task_type": "question"}}
Plan: {{"plan_description": "I will answer about my affiliation.", "steps": [{{"skill": "say", "args": {{"text": "I am from {team_affiliation}."}}}}]}}

Command: what is your name
Scene: {{"requires_scene": false, "task_type": "question"}}
Plan: {{"plan_description": "I will introduce the team.", "steps": [{{"skill": "say", "args": {{"text": "I am the robot from team {team_name}."}}}}]}}

Command: {command}
Scene: {scene_json}
Plan: """

# --- Stage 2: planner (benchmark few-shots + scene JSON, requires_scene=true) ---

PLANNER_PROMPT = """You are the planner for team {team_name}'s robot ({team_affiliation}, {team_country}).
Turn the command into a JSON plan using the scene JSON report.

The scene JSON is the authoritative world report — trust it completely. Do NOT override or second-guess it.

SKILLS (name(args) — purpose):
{skill_lines}

CHAIN OF THOUGHT — fill "reasoning" with these 4 steps BEFORE choosing steps:
1. LOCATIONS: use scene.locations known and unknown as-is.
2. PEOPLE/OBJECTS: use scene.people and scene.objects known and unknown as-is.
3. TASK: use scene.task_type and scene.has_find_intent as-is.
4. DECISION: refuse with say OR execute — name the rule and the skill sequence.

Rules (apply scene JSON as given):
- SEARCH: unknown people/objects OK when scene.has_find_intent=true; refuse if any required location is in scene.locations.unknown.
- INTERACTION without has_find_intent: refuse if scene.people.unknown is non-empty.
- INTERACTION with has_find_intent=true: go_to a scene.locations.known place first, find_person for scene.people.unknown, then guide/follow/give.
- NAVIGATION/MANIPULATION: refuse if any required location is in scene.locations.unknown.
- Output ONE JSON object: {{"reasoning": "1.LOCATIONS: ... 2.PEOPLE/OBJECTS: ... 3.TASK: ... 4.DECISION: ...", "plan_description": "...", "steps": [{{"skill": "...", "args": {{...}}}}]}}
- For say steps, "text" is only the spoken words.

EXAMPLES:

Command: lead Simone from the coatrack to the bathroom
Scene: {{"locations": {{"mentioned": ["coatrack", "bathroom"], "known": [], "unknown": ["coatrack", "bathroom"]}}, "people": {{"mentioned": ["simone"], "known": [], "unknown": ["simone"]}}, "objects": {{"mentioned": [], "known": [], "unknown": []}}, "task_type": "interaction", "has_find_intent": false, "required_skills": ["guide_person"]}}
Plan: {{"reasoning": "1.LOCATIONS: scene unknown=[coatrack,bathroom]. 2.PEOPLE: scene people.unknown=[simone]. 3.TASK: scene has_find_intent=false. 4.DECISION: refuse.", "plan_description": "I cannot escort Simone.", "steps": [{{"skill": "say", "args": {{"text": "I'm sorry, I don't know Simone or those locations."}}}}]}}

Command: meet Charlie in the kitchen and escort them to the bedroom
Scene: {{"locations": {{"mentioned": ["kitchen", "bedroom"], "known": ["kitchen", "bedroom"], "unknown": []}}, "people": {{"mentioned": ["charlie"], "known": [], "unknown": ["charlie"]}}, "objects": {{"mentioned": [], "known": [], "unknown": []}}, "task_type": "interaction", "has_find_intent": true, "required_skills": ["go_to_location", "find_person", "guide_person"]}}
Plan: {{"reasoning": "1.LOCATIONS: scene known=[kitchen,bedroom], unknown=[]. 2.PEOPLE: scene people.unknown=[charlie]. 3.TASK: scene has_find_intent=true. 4.DECISION: go_to kitchen, find charlie, guide to bedroom.", "plan_description": "I will find Charlie and escort them to the bedroom.", "steps": [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}, {{"skill": "find_person", "args": {{"name": "charlie", "location": "kitchen"}}}}, {{"skill": "guide_person", "args": {{"name": "charlie", "start": "kitchen", "end": "bedroom"}}}}]}}

Command: go to the waste basket then find a mustard and take it and put it on the refrigerator
Scene: {{"locations": {{"mentioned": ["waste basket", "refrigerator"], "known": [], "unknown": ["waste basket", "refrigerator"]}}, "people": {{"mentioned": [], "known": [], "unknown": []}}, "objects": {{"mentioned": ["mustard"], "known": ["mustard"], "unknown": []}}, "task_type": "manipulation", "has_find_intent": true, "required_skills": ["go_to_location", "find_object", "pick_up", "place_object"]}}
Plan: {{"reasoning": "1.LOCATIONS: waste basket and refrigerator unknown → cannot navigate. 2.OBJECTS: mustard known. 3.TASK: manipulation with find intent. 4.DECISION: required locations unknown → refuse.", "plan_description": "I cannot reach the waste basket or refrigerator.", "steps": [{{"skill": "say", "args": {{"text": "I'm sorry, I don't know where the waste basket or refrigerator is."}}}}]}}

Command: tell me what is the biggest object on the sofa
Scene: {{"locations": {{"mentioned": ["sofa"], "known": [], "unknown": ["sofa"]}}, "people": {{"mentioned": [], "known": [], "unknown": []}}, "objects": {{"mentioned": [], "known": [], "unknown": []}}, "task_type": "search", "has_find_intent": true, "required_skills": ["find_object_by_property"]}}
Plan: {{"reasoning": "1.LOCATIONS: sofa not in world → unknown. 2.OBJECTS: none specific. 3.TASK: search at unknown location. 4.DECISION: refuse.", "plan_description": "I cannot check the sofa.", "steps": [{{"skill": "say", "args": {{"text": "I'm sorry, the sofa is not a location I know."}}}}]}}

Command: locate a snack in the bathroom then take it and put it on the sofa
Scene: {{"locations": {{"mentioned": ["bathroom", "sofa"], "known": [], "unknown": ["bathroom", "sofa"]}}, "people": {{"mentioned": [], "known": [], "unknown": []}}, "objects": {{"mentioned": ["snack"], "known": [], "unknown": ["snack"]}}, "task_type": "manipulation", "has_find_intent": true, "required_skills": ["go_to_location", "find_object", "pick_up", "place_object"]}}
Plan: {{"reasoning": "1.LOCATIONS: bathroom and sofa unknown. 2.OBJECTS: snack unknown OK only if location known. 3.TASK: manipulation. 4.DECISION: bathroom unknown → refuse.", "plan_description": "I cannot go to the bathroom.", "steps": [{{"skill": "say", "args": {{"text": "I'm sorry, the bathroom is not a room I know."}}}}]}}

Command: go to the kitchen
Scene: {{"locations": {{"mentioned": ["kitchen"], "known": ["kitchen"], "unknown": []}}, "people": {{"mentioned": [], "known": [], "unknown": []}}, "objects": {{"mentioned": [], "known": [], "unknown": []}}, "task_type": "navigation", "has_find_intent": false, "required_skills": ["go_to_location"]}}
Plan: {{"reasoning": "1.LOCATIONS: kitchen known. 2.PEOPLE/OBJECTS: none. 3.TASK: navigation. 4.DECISION: go_to kitchen.", "plan_description": "I will go to the kitchen.", "steps": [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}]}}

Command: go to the bedroom find the mustard take it and bring it to the kitchen
Scene: {{"locations": {{"mentioned": ["bedroom", "kitchen"], "known": ["bedroom", "kitchen"], "unknown": []}}, "people": {{"mentioned": [], "known": [], "unknown": []}}, "objects": {{"mentioned": ["mustard"], "known": ["mustard"], "unknown": []}}, "task_type": "manipulation", "has_find_intent": true, "required_skills": ["go_to_location", "find_object", "pick_up", "place_object"]}}
Plan: {{"reasoning": "1.LOCATIONS: bedroom and kitchen known. 2.OBJECTS: mustard known in bedroom. 3.TASK: manipulation with find. 4.DECISION: bedroom→find→pick→kitchen→place.", "plan_description": "I will bring the mustard from the bedroom to the kitchen.", "steps": [{{"skill": "go_to_location", "args": {{"location": "bedroom"}}}}, {{"skill": "find_object", "args": {{"object": "mustard", "location": "bedroom"}}}}, {{"skill": "pick_up", "args": {{"object": "mustard"}}}}, {{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}, {{"skill": "place_object", "args": {{"location": "kitchen"}}}}]}}

Command: guide John from the kitchen to the living room
Scene: {{"locations": {{"mentioned": ["kitchen", "living room"], "known": ["kitchen", "living room"], "unknown": []}}, "people": {{"mentioned": ["john"], "known": ["john"], "unknown": []}}, "objects": {{"mentioned": [], "known": [], "unknown": []}}, "task_type": "interaction", "has_find_intent": false, "required_skills": ["guide_person"]}}
Plan: {{"reasoning": "1.LOCATIONS: kitchen and living room known. 2.PEOPLE: john known. 3.TASK: interaction, no find needed. 4.DECISION: go_to kitchen, guide john.", "plan_description": "I will guide John to the living room.", "steps": [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}, {{"skill": "guide_person", "args": {{"name": "john", "start": "kitchen", "end": "living room"}}}}]}}

Command: meet Emma in the living room and escort her to the kitchen
Scene: {{"locations": {{"mentioned": ["living room", "kitchen"], "known": ["living room", "kitchen"], "unknown": []}}, "people": {{"mentioned": ["emma"], "known": ["emma"], "unknown": []}}, "objects": {{"mentioned": [], "known": [], "unknown": []}}, "task_type": "interaction", "has_find_intent": true, "required_skills": ["go_to_location", "find_person", "guide_person"]}}
Plan: {{"reasoning": "1.LOCATIONS: living room and kitchen known. 2.PEOPLE: emma known, meet implies find. 3.TASK: interaction, has_find_intent=true. 4.DECISION: go_to living room, find emma, guide to kitchen.", "plan_description": "I will find Emma and escort her to the kitchen.", "steps": [{{"skill": "go_to_location", "args": {{"location": "living room"}}}}, {{"skill": "find_person", "args": {{"name": "emma", "location": "living room"}}}}, {{"skill": "guide_person", "args": {{"name": "emma", "start": "living room", "end": "kitchen"}}}}]}}

Command: find the apple in the kitchen
Scene: {{"locations": {{"mentioned": ["kitchen"], "known": ["kitchen"], "unknown": []}}, "people": {{"mentioned": [], "known": [], "unknown": []}}, "objects": {{"mentioned": ["apple"], "known": ["apple"], "unknown": []}}, "task_type": "search", "has_find_intent": true, "required_skills": ["go_to_location", "find_object"]}}
Plan: {{"reasoning": "1.LOCATIONS: kitchen known. 2.OBJECTS: apple known in kitchen. 3.TASK: search. 4.DECISION: go_to kitchen, find apple.", "plan_description": "I will find the apple in the kitchen.", "steps": [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}, {{"skill": "find_object", "args": {{"object": "apple", "location": "kitchen"}}}}]}}

Now plan this command (think step by step in reasoning, then output the plan):
Command: {command}
Scene: {scene_json}
Plan: """

# --- Stage 3: plan critic (LLM self-correction) ---

PLAN_CRITIC_PROMPT = """You are a strict plan VALIDATOR for a robot — not an executor.
Check the draft plan against the scene JSON only. Trust the scene as the authoritative world report.
If the draft is correct, return it EXACTLY unchanged (same steps, same text).
NEVER turn a refusal (single say step) into go_to/find/guide steps. Refusal is often correct.

SKILLS (name(args) — purpose):
{skill_lines}

Rules (use scene JSON):
- Refuse (say only) when scene.locations.unknown is non-empty for a required place, or scene.people.unknown is non-empty with scene.has_find_intent=false.
- find_person/find_object OK for scene.people/objects unknown ONLY when scene.has_find_intent=true AND scene.locations.unknown is empty.
- Output ONLY JSON: {{"plan_description": "...", "steps": [{{"skill": "...", "args": {{...}}}}]}}

EXAMPLES:

Draft: {{"plan_description": "I cannot escort Simone.", "steps": [{{"skill": "say", "args": {{"text": "I'm sorry, I don't know Simone or those locations."}}}}]}}
→ return UNCHANGED (refusal is correct)

Draft: {{"plan_description": "I cannot check the sofa.", "steps": [{{"skill": "say", "args": {{"text": "I'm sorry, the sofa is not a location I know."}}}}]}}
→ return UNCHANGED

Draft with go_to waste basket: {{"steps": [{{"skill": "go_to_location", "args": {{"location": "waste basket"}}}}]}}
→ {{"plan_description": "I cannot reach the waste basket.", "steps": [{{"skill": "say", "args": {{"text": "I'm sorry, I don't know where the waste basket is."}}}}]}}

Draft: {{"plan_description": "I will go to the kitchen.", "steps": [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}]}}
→ return UNCHANGED (feasible navigation is correct)

Command: {command}
Scene: {scene_json}
Draft plan: {draft_plan}
Corrected plan: """


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


def load_team(node):
    path = _pkg_config(node, "team.yaml")
    if not os.path.exists(path):
        return {"name": "LASR", "affiliation": "King's College London", "country": "United Kingdom"}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return data.get("team", {})


def _format_objects(objects):
    if not objects:
        return "none"
    lines = [
        "Grabbable objects ONLY (not furniture/surfaces/appliances):",
    ]
    for name, obj in objects.items():
        aliases = obj.get("aliases") or []
        alias_txt = f", aliases: {', '.join(aliases)}" if aliases else ""
        lines.append(
            f"- {name}: {obj.get('category', '?')}, in {obj.get('location', '?')}, "
            f"{obj.get('description', '')}{alias_txt}"
        )
    return "\n".join(lines)


def _format_people(people):
    return ", ".join(
        f"{name} ({info.get('gender', '?')})" for name, info in people.items()
    ) or "none"


def _parse_json(raw: str) -> dict:
    start = raw.find("{")
    end = raw.rfind("}") + 1
    if start == -1 or end == 0:
        return {}
    return json.loads(raw[start:end])


def _extract_plan(parsed: dict) -> tuple[str, list, str]:
    """Accept planner or critic JSON; tolerate nested 'plan' key."""
    if "plan" in parsed and isinstance(parsed["plan"], dict):
        parsed = parsed["plan"]
    steps = parsed.get("steps", [])
    plan_description = parsed.get("plan_description", "")
    reasoning = parsed.get("reasoning", "")
    if not steps:
        raise ValueError("No steps in plan")
    return plan_description, steps, reasoning


def _requires_scene(scene: dict) -> bool:
    """True when the command needs spatial scene reasoning (locations/people/objects)."""
    if "requires_scene" in scene:
        return bool(scene["requires_scene"])
    if scene.get("task_type") == "question":
        for key in ("locations", "people", "objects"):
            block = scene.get(key, {})
            if block.get("mentioned"):
                return True
        return False
    return True


def _plan_for_critic(draft_raw: str) -> str:
    """Strip chain-of-thought; critic validates only plan_description + steps."""
    parsed = _parse_json(draft_raw)
    clean = {
        "plan_description": parsed.get("plan_description", ""),
        "steps": parsed.get("steps", []),
    }
    return json.dumps(clean, ensure_ascii=False)


class QueryLLM(yasmin.State):
    """Scene analyst → planner (or general planner) → optional critic."""

    def __init__(self, node):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("sequence")
        self.add_output_key("skill")
        self.add_output_key("skill_args")
        self.add_output_key("plan_description")
        self.add_output_key("steps")
        self.add_output_key("reports")
        self.add_output_key("scene")
        self.add_output_key("planner_reasoning")
        self.add_output_key("raw_planner_output")
        self.add_output_key("raw_critic_output")
        self.add_output_key("timing_sec")
        self.node = node

        self.locations = load_locations(node)
        self.objects = load_objects(node)
        self.people = load_people(node)
        self.team = load_team(node)
        self.skills_text = load_skills_text(node)
        self.skill_lines = compact_skill_lines(self.skills_text)

        model = node.get_parameter("llm_model").value
        host = node.get_parameter("llm_host").value

        self.agent = Agent(model=model, host=host)

        self.node.get_logger().info(f"Loading Ollama model into memory: {model}")
        self.agent.warmup()
        self.node.get_logger().info("QueryLLM ready (analyst + planner + critic).")

    def _analyze_scene(self, command: str) -> dict:
        prompt = SCENE_ANALYST_PROMPT.format(
            locations=", ".join(self.locations.keys()) or "none",
            object_names=", ".join(self.objects.keys()) or "none",
            people_names=", ".join(self.people.keys()) or "none",
            objects=_format_objects(self.objects),
            people=_format_people(self.people),
            command=command,
        )
        raw = self.agent.query_json(prompt, max_tokens=1024)
        self.node.get_logger().info(f"Scene analyst: {raw}")
        scene = _parse_json(raw)
        if not scene.get("locations"):
            raise ValueError("Invalid scene JSON from analyst")
        reasoning = scene.pop("reasoning", "")
        if reasoning:
            self.node.get_logger().info(f"Analyst reasoning: {reasoning}")
        self.node.get_logger().info(f"Scene report: {json.dumps(scene, ensure_ascii=False)}")
        return scene

    def _plan_general(self, command: str, scene: dict) -> str:
        scene_json = json.dumps(scene, ensure_ascii=False)
        prompt = GENERAL_PLANNER_PROMPT.format(
            team_name=self.team.get("name", "LASR"),
            team_affiliation=self.team.get("affiliation", ""),
            team_country=self.team.get("country", ""),
            command=command,
            scene_json=scene_json,
        )
        raw = self.agent.query_json(prompt, max_tokens=512)
        self.node.get_logger().info(f"General planner output: {raw}")
        return raw

    def _plan(self, command: str, scene: dict) -> str:
        scene_json = json.dumps(scene, ensure_ascii=False)
        prompt = PLANNER_PROMPT.format(
            team_name=self.team.get("name", "LASR"),
            team_affiliation=self.team.get("affiliation", ""),
            team_country=self.team.get("country", ""),
            skill_lines=self.skill_lines,
            command=command,
            scene_json=scene_json,
        )
        raw = self.agent.query_json(prompt, max_tokens=1536)
        self.node.get_logger().info(f"Planner output: {raw}")
        return raw

    def _critique(self, command: str, scene: dict, draft_raw: str) -> str:
        scene_json = json.dumps(scene, ensure_ascii=False)
        prompt = PLAN_CRITIC_PROMPT.format(
            skill_lines=self.skill_lines,
            command=command,
            scene_json=scene_json,
            draft_plan=_plan_for_critic(draft_raw),
        )
        raw = self.agent.query_json(prompt)
        self.node.get_logger().info(f"Critic output: {raw}")
        return raw

    def execute(self, blackboard):
        t_start = time.perf_counter()
        command = blackboard["sequence"].strip()
        self.node.get_logger().info(f"LLM pipeline query: '{command}'")

        t_analyst_start = time.perf_counter()
        try:
            scene = self._analyze_scene(command)
        except Exception as e:
            self.node.get_logger().error(f"Scene analyst failed: {e}")
            blackboard["timing_sec"] = {
                "analyst": round(time.perf_counter() - t_analyst_start, 3),
                "planner": 0.0,
                "critic": 0.0,
                "total": round(time.perf_counter() - t_start, 3),
            }
            return "failed"

        analyst_sec = time.perf_counter() - t_analyst_start
        blackboard["scene"] = scene
        blackboard["reports"] = scene
        fast_path = not _requires_scene(scene)
        if fast_path:
            self.node.get_logger().info("Fast path: general question — skipping critic")

        t_planner_start = time.perf_counter()
        try:
            if fast_path:
                draft_raw = self._plan_general(command, scene)
            else:
                draft_raw = self._plan(command, scene)
            blackboard["raw_planner_output"] = draft_raw
        except Exception as e:
            self.node.get_logger().warn(f"Planner failed: {e}")
            blackboard["skill"] = "say"
            blackboard["skill_args"] = {"text": "I could not generate a plan for that command."}
            blackboard["plan_description"] = ""
            blackboard["steps"] = []
            blackboard["raw_planner_output"] = ""
            blackboard["raw_critic_output"] = ""
            blackboard["timing_sec"] = {
                "analyst": round(analyst_sec, 3),
                "planner": round(time.perf_counter() - t_planner_start, 3),
                "critic": 0.0,
                "fast_path": fast_path,
                "total": round(time.perf_counter() - t_start, 3),
            }
            return "succeeded"

        planner_sec = time.perf_counter() - t_planner_start

        t_critic_start = time.perf_counter()
        final_raw = draft_raw
        if fast_path:
            blackboard["raw_critic_output"] = ""
            critic_sec = 0.0
        else:
            try:
                final_raw = self._critique(command, scene, draft_raw)
                blackboard["raw_critic_output"] = final_raw
            except Exception as e:
                self.node.get_logger().warn(f"Critic failed, using draft plan: {e}")
                blackboard["raw_critic_output"] = ""
            critic_sec = time.perf_counter() - t_critic_start

        planner_reasoning = ""
        try:
            draft_parsed = _parse_json(draft_raw)
            planner_reasoning = draft_parsed.get("reasoning", "")
        except Exception:
            pass

        try:
            plan_description, steps, _ = _extract_plan(_parse_json(final_raw))
        except Exception as e:
            self.node.get_logger().warn(f"Plan parse failed: {e}")
            blackboard["skill"] = "say"
            blackboard["skill_args"] = {"text": "I could not generate a plan for that command."}
            blackboard["plan_description"] = ""
            blackboard["steps"] = []
            blackboard["planner_reasoning"] = planner_reasoning
            blackboard["timing_sec"] = {
                "analyst": round(analyst_sec, 3),
                "planner": round(planner_sec, 3),
                "critic": round(critic_sec, 3),
                "fast_path": fast_path,
                "total": round(time.perf_counter() - t_start, 3),
            }
            return "succeeded"

        blackboard["plan_description"] = plan_description
        blackboard["steps"] = steps
        blackboard["planner_reasoning"] = planner_reasoning
        blackboard["skill"] = steps[0]["skill"]
        blackboard["skill_args"] = steps[0].get("args", {})
        blackboard["timing_sec"] = {
            "analyst": round(analyst_sec, 3),
            "planner": round(planner_sec, 3),
            "critic": round(critic_sec, 3),
            "fast_path": fast_path,
            "total": round(time.perf_counter() - t_start, 3),
        }
        if planner_reasoning:
            self.node.get_logger().info(f"Planner reasoning: {planner_reasoning}")
        self.node.get_logger().info(
            f"Plan: {plan_description} | Steps: {steps} | "
            f"Timing(s): analyst={blackboard['timing_sec']['analyst']} "
            f"planner={blackboard['timing_sec']['planner']} "
            f"critic={blackboard['timing_sec']['critic']} "
            f"total={blackboard['timing_sec']['total']}"
        )
        return "succeeded"
