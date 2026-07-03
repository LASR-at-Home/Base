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


# -----------------------------------------------------------------------
# CALL 1 — split the task into its actions
# -----------------------------------------------------------------------
SPLIT_PROMPT = """Split this robot command into its separate actions, in order.
Split on connectors like "then"/"and" when they join actions. Keep each action's
own words (objects, people, places) inside it. Never drop or merge actions.

Output ONLY JSON: {{"actions": ["<action 1>", "<action 2>", ...]}}

Examples:
- "go to the kitchen then find an apple and take it and bring it to me"
  -> {{"actions": ["go to the kitchen", "find an apple", "take it", "bring it to me"]}}
- "locate a standing person in the living room and follow them to the laundry table"
  -> {{"actions": ["locate a standing person in the living room", "follow them to the laundry table"]}}
- "tell me how many fruits there are on the tv stand"
  -> {{"actions": ["tell me how many fruits there are on the tv stand"]}}

Command: {command}
JSON: """


def split_actions(command: str) -> list:
    return _query(SPLIT_PROMPT.format(command=command)).get("actions", [command])


# -----------------------------------------------------------------------
# CALL 2 — classify: template for action 1, one followup per later action
# -----------------------------------------------------------------------
MATCH_PROMPT = """You are a command classifier
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

The command has already been split into its actions (one per line, in order).
Classify: the FIRST action is the template; EVERY following action is exactly ONE followup.
The number of followups MUST equal the number of actions after the first — never drop one.

Output ONLY JSON:
{{"template": "<template_name>", "followup": "<one followup per remaining action joined by '>', or empty string>", "slots": {{"<slot>": "<value>", ...}}}}

CRITICAL: fill the slots for the template AND for every followup, using the slot names
shown in their placeholders. Every {{placeholder}} of the chosen template/followups that
appears in the actions MUST have a slot. Use the exact slot names (posePers for
sitting/standing/lying person, gestPers for waving/pointing/raising person, loc2/room2
for the destination of followPrsToRoom/guidePrsToBeacon, plcmtLoc2 for placeObjOnPlcmt).

Examples (with slots):
- actions: ["go to the kitchen", "find an apple", "take it", "throw it in the trash"]
  -> {{"template": "goToLoc", "followup": "findObj>takeObj>putObjInTrash", "slots": {{"loc": "kitchen", "obj": "apple"}}}}
- actions: ["take a coke from the desk", "bring it to me"]
  -> {{"template": "takeObjFromPlcmt", "followup": "deliverObjToMe", "slots": {{"obj": "coke", "plcmtLoc": "desk"}}}}
- actions: ["find a toy in the kitchen", "take it", "place it on the shelf"]
  -> {{"template": "findObjInRoom", "followup": "takeObj>placeObjOnPlcmt", "slots": {{"obj": "toy", "room": "kitchen", "plcmtLoc2": "shelf"}}}}
- actions: ["locate a standing person in the living room", "follow them to the laundry table"]
  -> {{"template": "findPrsInRoom", "followup": "followPrsToRoom", "slots": {{"posePers": "standing person", "room": "living room", "loc2": "laundry table"}}}}
- actions: ["meet Jane in the office", "follow them"]
  -> {{"template": "meetPrsAtBeac", "followup": "followPrs", "slots": {{"name": "Jane", "room": "office"}}}}
- actions: ["greet the waving person in the bedroom", "guide them to the sofa"]
  -> {{"template": "findPrsInRoom", "followup": "guidePrsToBeacon", "slots": {{"gestPers": "waving person", "room": "bedroom", "loc2": "sofa"}}}}
- actions: ["bring me a soju from the cabinet"]
  -> {{"template": "bringMeObjFromPlcmt", "followup": "", "slots": {{"obj": "soju", "plcmtLoc": "cabinet"}}}}

Only include slots actually present in the actions. Verbs are synonyms
(grab/take/get/fetch, bring/give/deliver, go/navigate, find/locate/look for, etc.).

Actions:
{actions}
JSON: """


def match_template(actions: list) -> dict:
    actions_text = "\n".join(f"{i}. {a}" for i, a in enumerate(actions, 1))
    return _query(MATCH_PROMPT.format(actions=actions_text))


if __name__ == "__main__":
    task = " ".join(sys.argv[1:]) or "grab the apple from the dinner table and bring it to the waving person in the kitchen"

    actions = split_actions(task)
    print("=== CALL 1: ACTIONS ===")
    print(json.dumps(actions, indent=2))

    result = match_template(actions)
    print("\n=== CALL 2: TEMPLATE MATCH ===")
    print(json.dumps(result, indent=2))