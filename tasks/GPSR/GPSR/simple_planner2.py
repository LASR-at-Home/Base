"""Template-matching planner: instead of free-form planning, classify the
command back into the gpsr_commands.py template that generated it.

GPSR commands are produced by CommandGenerator from a closed set of templates
+ followups + slot values, so recognition is the inverse problem: one LLM call
that picks the template, the followup (if any), and fills the slots.

Output example:
{
  "template": "takeObjFromPlcmt",
  "followup": "deliverObjToPrsInRoom",
  "slots": {"obj": "apple", "plcmtLoc": "dinner table",
            "gestPers": "waving person", "room": "kitchen"}
}

Standalone, no ROS required — calls ollama directly.

Run: python3 simple_planner2.py "grab the apple from the dinner table and bring it to the waving person in the kitchen"
"""

import json
import sys

import ollama

MODEL = "gemma3"


def _parse_json(raw: str) -> dict:
    start = raw.find("{")
    end = raw.rfind("}") + 1
    if start == -1 or end == 0:
        return {}
    return json.loads(raw[start:end])


def _query(prompt: str) -> dict:
    response = ollama.chat(
        model=MODEL,
        messages=[{"role": "user", "content": prompt}],
        format="json",
    )
    return _parse_json(response["message"]["content"])


# Templates and followups mirror CommandGenerator in
# CommandGenerator/src/robocupathome_generator/gpsr_commands.py — keep in sync.
MATCH_PROMPT = """You are a RoboCup@Home GPSR command classifier.
Every command was generated from exactly ONE of these templates (shown with their slot placeholders):

goToLoc: "go to the {{loc|room}} then <FOLLOWUP>"
takeObjFromPlcmt: "take a {{obj|singCat}} from the {{plcmtLoc}} and <FOLLOWUP>"
findPrsInRoom: "find a {{gestPers|posePers}} in the {{room}} and <FOLLOWUP>"
findObjInRoom: "find a {{obj|singCat}} in the {{room}} then <FOLLOWUP>"
meetPrsAtBeac: "meet {{name}} in the {{room}} and <FOLLOWUP>"
countObjOnPlcmt: "tell me how many {{plurCat}} there are on the {{plcmtLoc}}"
countPrsInRoom: "tell me how many {{gestPersPlur|posePersPlur}} are in the {{room}}"
tellPrsInfoInLoc: "tell me the {{persInfo}} of the person in the {{room}} / at the {{loc}}"
tellObjPropOnPlcmt: "tell me what is the {{objComp}} object on the {{plcmtLoc}}"
talkInfoToGestPrsInRoom: "tell {{talk}} to the {{gestPers}} in the {{room}}"
followNameFromBeacToRoom: "follow {{name}} from the {{loc}} to the {{room}}"
guideNameFromBeacToBeac: "guide {{name}} from the {{loc}} to the {{loc2|room}}"
guidePrsFromBeacToBeac: "guide the {{gestPers|posePers}} from the {{loc}} to the {{loc2|room}}"
guideClothPrsFromBeacToBeac: "guide the person wearing a {{colorClothe}} from the {{loc}} to the {{loc2|room}}"
bringMeObjFromPlcmt: "bring me a {{obj}} from the {{plcmtLoc}}"
tellCatPropOnPlcmt: "tell me what is the {{objComp}} {{singCat}} on the {{plcmtLoc}}"
greetClothDscInRm: "greet the person wearing a {{colorClothe}} in the {{room}} and <FOLLOWUP>"
greetNameInRm: "greet {{name}} in the {{room}} and <FOLLOWUP>"
meetNameAtLocThenFindInRm: "meet {{name}} at the {{loc}} then find them in the {{room}}"
countClothPrsInRoom: "tell me how many people in the {{room}} are wearing {{colorClothes}}"
tellPrsInfoAtLocToPrsAtLoc: "tell the {{persInfo}} of the person at the {{loc}} to the person at the {{loc2}}"
followPrsAtLoc: "follow the {{gestPers|posePers}} in the {{room}} / at the {{loc}}"

<FOLLOWUP> (when present) is exactly ONE of:
findObj: "find a {{obj|singCat}} and <FOLLOWUP>"
findPrs: "find the {{gestPers|posePers}} and <FOLLOWUP>"
meetName: "meet {{name}} and <FOLLOWUP>"
placeObjOnPlcmt: "place it on the {{plcmtLoc2}}"
putObjInTrash: "throw it in the trash"
deliverObjToMe: "bring it to me"
deliverObjToPrsInRoom: "bring it to the {{gestPers|posePers}} in the {{room}}"
deliverObjToNameAtBeac: "bring it to {{name}} in the {{room}}"
talkInfo: "say {{talk}}"
followPrs: "follow them"
followPrsToRoom: "follow them to the {{loc2|room2}}"
guidePrsToBeacon: "guide them to the {{loc2|room2}}"
takeObj: "take it and <FOLLOWUP>"

Slot meanings:
obj = object name | singCat/plurCat = object category | plcmtLoc/plcmtLoc2 = placement surface
loc/loc2 = location/beacon | room/room2 = room name | name = person name
gestPers = gesture person descriptor (e.g. "waving person") | posePers = pose person descriptor (e.g. "lying person")
gestPersPlur/posePersPlur = plural descriptors | persInfo = name/pose/gesture
objComp = biggest/largest/smallest/heaviest/lightest/thinnest | talk = phrase to say
colorClothe(s) = color + garment (e.g. "grey shirt")

Classify the command. Output ONLY JSON:
{{"template": "<template_name>", "followup": "<full followup chain joined by '>', or empty string>", "slots": {{"<slot>": "<value>", ...}}}}

IMPORTANT: followup must list the FULL chain of followups, one per clause after the
template, joined by '>'. Followups containing <FOLLOWUP> (findObj, findPrs, meetName,
takeObj) are ALWAYS followed by another followup in the chain — never end the chain there.
Examples:
- "go to the X then find a Y and take it and throw it in the trash" -> template goToLoc, followup "findObj>takeObj>putObjInTrash"
- "take a Y from the X and bring it to me" -> template takeObjFromPlcmt, followup "deliverObjToMe"
- "find a Y in the X then take it and place it on the Z" -> template findObjInRoom, followup "takeObj>placeObjOnPlcmt"
- "meet NAME in the X and follow them" -> template meetPrsAtBeac, followup "followPrs"
- "meet NAME in the X and tell something about yourself" -> template meetPrsAtBeac, followup "talkInfo"

NEVER add a followup for an action that is not in the command: each followup must match
an actual clause. If the command's last clause is covered by talkInfo/followPrs/putObjInTrash/etc.,
the chain ENDS there — do not append extra followups.

Only include slots actually present in the command. Verbs are synonyms
(grab/take/get/fetch, bring/give/deliver, go/navigate, find/locate/look for, etc.).

Command: {command}
JSON: """


def match_template(command: str) -> dict:
    return _query(MATCH_PROMPT.format(command=command))


# -----------------------------------------------------------------------
# Deterministic expansion: template/followup + slots -> skill steps
# (skill names/args from config/skills.yaml, executed by dispatch_skill.py)
# -----------------------------------------------------------------------
# Each entry is a list of (skill, {arg: slot_or_literal}) — "$x" pulls slot x,
# "$x|y" tries slot x then y. Missing slots resolve to "".
TEMPLATE_STEPS = {
    "goToLoc": [("go_to_location", {"location": "$loc|room"})],
    "takeObjFromPlcmt": [
        ("go_to_location", {"location": "$plcmtLoc"}),
        ("find_object", {"object": "$obj|singCat", "location": "$plcmtLoc"}),
        ("pick_up", {"object": "$obj|singCat"}),
    ],
    "findPrsInRoom": [
        ("go_to_location", {"location": "$room"}),
        ("find_person", {"gesture": "$gestPers|posePers|gestPers_posePers", "location": "$room"}),
    ],
    "findObjInRoom": [
        ("go_to_location", {"location": "$room"}),
        ("find_object", {"object": "$obj|singCat", "location": "$room"}),
    ],
    "meetPrsAtBeac": [
        ("go_to_location", {"location": "$room"}),
        ("find_person", {"name": "$name", "location": "$room"}),
    ],
    "countObjOnPlcmt": [
        ("go_to_location", {"location": "$plcmtLoc"}),
        ("count_objects", {"object": "$plurCat|plurcat|obj|singCat", "location": "$plcmtLoc"}),
        ("go_to_location", {"location": "instruction point"}),
        ("say", {"text": "The number of $plurCat|plurcat|obj|singCat on the $plcmtLoc is [result]."}),
    ],
    "countPrsInRoom": [
        ("go_to_location", {"location": "$room"}),
        ("count_people", {"pose": "$posePersPlur", "gesture": "$gestPersPlur", "location": "$room"}),
        ("go_to_location", {"location": "instruction point"}),
        ("say", {"text": "The number of people in the $room is [result]."}),
    ],
    "tellPrsInfoInLoc": [
        ("go_to_location", {"location": "$room|loc"}),
        ("get_person_info", {"info": "$persInfo", "location": "$room|loc"}),
        ("go_to_location", {"location": "instruction point"}),
        ("say", {"text": "The $persInfo of the person is [result]."}),
    ],
    "tellObjPropOnPlcmt": [
        ("go_to_location", {"location": "$plcmtLoc"}),
        ("find_object_by_property", {"property": "$objComp", "location": "$plcmtLoc"}),
        ("go_to_location", {"location": "instruction point"}),
        ("say", {"text": "The $objComp object on the $plcmtLoc is [result]."}),
    ],
    "talkInfoToGestPrsInRoom": [
        ("go_to_location", {"location": "$room"}),
        ("find_person", {"gesture": "$gestPers|posePers|gestPers_posePers", "location": "$room"}),
        ("say", {"text": "$talk"}),
    ],
    "followNameFromBeacToRoom": [
        ("go_to_location", {"location": "$loc"}),
        ("find_person", {"name": "$name", "location": "$loc"}),
        ("follow_person", {}),
    ],
    "guideNameFromBeacToBeac": [
        ("guide_person", {"name": "$name", "start": "$loc", "end": "$loc2|room"}),
    ],
    "guidePrsFromBeacToBeac": [
        ("go_to_location", {"location": "$loc"}),
        ("find_person", {"gesture": "$gestPers|posePers|gestPers_posePers", "location": "$loc"}),
        ("guide_person", {"start": "$loc", "end": "$loc2|room"}),
    ],
    "guideClothPrsFromBeacToBeac": [
        ("go_to_location", {"location": "$loc"}),
        ("find_person", {"clothes": "$colorClothe", "location": "$loc"}),
        ("guide_person", {"start": "$loc", "end": "$loc2|room"}),
    ],
    "bringMeObjFromPlcmt": [
        ("go_to_location", {"location": "$plcmtLoc"}),
        ("find_object", {"object": "$obj", "location": "$plcmtLoc"}),
        ("pick_up", {"object": "$obj"}),
        ("go_to_location", {"location": "instruction point"}),
        ("give_to_person", {"object": "$obj"}),
    ],
    "tellCatPropOnPlcmt": [
        ("go_to_location", {"location": "$plcmtLoc"}),
        ("find_object_by_property", {"property": "$objComp", "object": "$singCat", "location": "$plcmtLoc"}),
        ("go_to_location", {"location": "instruction point"}),
        ("say", {"text": "The $objComp $singCat on the $plcmtLoc is [result]."}),
    ],
    "greetClothDscInRm": [
        ("go_to_location", {"location": "$room"}),
        ("find_person", {"clothes": "$colorClothe", "location": "$room"}),
        ("say", {"text": "Hello!"}),
    ],
    "greetNameInRm": [
        ("go_to_location", {"location": "$room"}),
        ("find_person", {"name": "$name", "location": "$room"}),
        ("say", {"text": "Hello $name!"}),
    ],
    "meetNameAtLocThenFindInRm": [
        ("go_to_location", {"location": "$loc"}),
        ("find_person", {"name": "$name", "location": "$loc"}),
        ("go_to_location", {"location": "$room"}),
        ("find_person", {"name": "$name", "location": "$room"}),
    ],
    "countClothPrsInRoom": [
        ("go_to_location", {"location": "$room"}),
        ("count_people", {"clothes": "$colorClothes", "location": "$room"}),
        ("go_to_location", {"location": "instruction point"}),
        ("say", {"text": "The number of people wearing $colorClothes in the $room is [result]."}),
    ],
    "tellPrsInfoAtLocToPrsAtLoc": [
        ("go_to_location", {"location": "$loc"}),
        ("get_person_info", {"info": "$persInfo", "location": "$loc"}),
        ("go_to_location", {"location": "$loc2"}),
        ("find_person", {"location": "$loc2"}),
        ("say", {"text": "The $persInfo of the person at the $loc is [result]."}),
    ],
    "followPrsAtLoc": [
        ("go_to_location", {"location": "$room|loc"}),
        ("find_person", {"gesture": "$gestPers|posePers|gestPers_posePers", "location": "$room|loc"}),
        ("follow_person", {}),
    ],
}

FOLLOWUP_STEPS = {
    "findObj": [("find_object", {"object": "$obj|singCat", "location": "$loc|room"})],
    "findPrs": [("find_person", {"gesture": "$gestPers|posePers|gestPers_posePers", "location": "$loc|room"})],
    "meetName": [("find_person", {"name": "$name", "location": "$loc|room"})],
    "placeObjOnPlcmt": [
        ("go_to_location", {"location": "$plcmtLoc2"}),
        ("place_object", {"object": "$obj|singCat", "location": "$plcmtLoc2"}),
    ],
    "putObjInTrash": [
        ("go_to_location", {"location": "trash bin"}),
        ("place_object", {"object": "$obj|singCat", "location": "trash bin"}),
    ],
    "deliverObjToMe": [
        ("go_to_location", {"location": "instruction point"}),
        ("give_to_person", {"object": "$obj|singCat"}),
    ],
    "deliverObjToPrsInRoom": [
        ("go_to_location", {"location": "$room"}),
        ("find_person", {"gesture": "$gestPers|posePers|gestPers_posePers", "location": "$room"}),
        ("give_to_person", {"object": "$obj|singCat"}),
    ],
    "deliverObjToNameAtBeac": [
        ("go_to_location", {"location": "$room"}),
        ("find_person", {"name": "$name", "location": "$room"}),
        ("give_to_person", {"object": "$obj|singCat"}),
    ],
    "talkInfo": [("say", {"text": "$talk"})],
    "followPrs": [("follow_person", {})],
    "followPrsToRoom": [("follow_person", {})],
    "guidePrsToBeacon": [("guide_person", {"end": "$loc2|room2"})],
    "takeObj": [("pick_up", {"object": "$obj|singCat"})],
}


import re as _re


def _fill(value: str, slots: dict) -> str:
    """Resolve "$a|b" slot references (whole-value or inline in literals)."""

    def resolve(ref: str) -> str:
        for key in ref.split("|"):
            if slots.get(key):
                return str(slots[key])
        return ""

    if value.startswith("$") and _re.fullmatch(r"\$\w+(\|\w+)*", value):
        return resolve(value[1:])
    return _re.sub(r"\$(\w+(?:\|\w+)*)", lambda m: resolve(m.group(1)), value)


def expand(match: dict) -> list:
    """Template match -> executable skill steps."""
    import difflib

    slots = match.get("slots", {})

    # LLMs sometimes invent near-miss template names — snap to the closest real one
    template = match.get("template", "")
    if template not in TEMPLATE_STEPS:
        close = difflib.get_close_matches(template, TEMPLATE_STEPS, n=1, cutoff=0.6)
        template = close[0] if close else ""

    followup = match.get("followup", "") or ""
    # drop followup names that don't exist (keep the valid rest of the chain)
    chain = [f for f in followup.split(">") if f in FOLLOWUP_STEPS]

    # a terminal followup ends the command — anything after it is LLM noise
    NON_TERMINAL = {"findObj", "findPrs", "meetName", "takeObj"}
    for j, f in enumerate(chain):
        if f not in NON_TERMINAL:
            chain = chain[: j + 1]
            break

    parts = [template] + chain

    # followups that hand the object over — require it to be in hand first
    DELIVERY_PARTS = {"placeObjOnPlcmt", "putObjInTrash", "deliverObjToMe",
                      "deliverObjToPrsInRoom", "deliverObjToNameAtBeac"}

    steps = []
    for i, part in enumerate(parts):
        table = TEMPLATE_STEPS if i == 0 else FOLLOWUP_STEPS

        # repair chains where the LLM skipped the takeObj followup: the object
        # must be found and picked up (where it is) before any delivery part
        if part in DELIVERY_PARTS:
            done = {s["skill"] for s in steps}
            obj = _fill("$obj|singCat", slots)
            if "find_object" not in done:
                steps.append({"skill": "find_object", "args": {"object": obj}})
            if "pick_up" not in done:
                steps.append({"skill": "pick_up", "args": {"object": obj}})

        for skill, arg_spec in table.get(part, []):
            args = {k: _fill(v, slots) for k, v in arg_spec.items()}
            args = {k: v for k, v in args.items() if v}
            # skip duplicate consecutive navigation to the same place
            if (skill == "go_to_location" and steps
                    and steps[-1]["skill"] == "go_to_location"
                    and steps[-1]["args"] == args):
                continue
            steps.append({"skill": skill, "args": args})
    return steps


if __name__ == "__main__":
    task = " ".join(sys.argv[1:]) or "grab the apple from the dinner table and bring it to the waving person in the kitchen"
    result = match_template(task)
    print("=== TEMPLATE MATCH (LLM) ===")
    print(json.dumps(result, indent=2))

    steps = expand(result)
    print("\n=== SKILL STEPS (deterministic) ===")
    print(json.dumps({
        "skill": steps[0]["skill"] if steps else "say",
        "skill_args": steps[0]["args"] if steps else {"text": "I could not parse that command."},
        "plan_description": task,
        "steps": steps,
    }, indent=2))
