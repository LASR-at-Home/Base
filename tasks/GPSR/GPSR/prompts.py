import json

SKILL_SELECTOR_PROMPT = """You are the skill selector for a robot.
Given a command, pick which skills from the list below are needed to execute it.
Output ONLY JSON. No explanation.

Available skills:
{skill_lines}

Output schema:
{{
  "can_do": true,
  "reason": "<one sentence why>",
  "selected_skills": ["skill_name", ...]
}}

RULES:
- selected_skills contains ONLY the bare skill name (e.g. "find_object"), never arguments or signatures (never "find_object(apple, kitchen)").
- can_do=false ONLY when NO skill matches the kind of action (e.g. cooking, flying).
- Missing locations/objects/people are NOT a reason for can_do=false — the planner handles those.
- Any question, greeting, or request for information: can_do=true, selected_skills=["say"].
- If the command needs navigation AND another action, include "go_to_location" in selected_skills.

EXAMPLES:
Command: find the apple in the kitchen
JSON: {{"can_do": true, "reason": "need to navigate and search", "selected_skills": ["go_to_location", "find_object"]}}

Command: count the people in the living room
JSON: {{"can_do": true, "reason": "need to navigate and count", "selected_skills": ["go_to_location", "count_people"]}}

Command: how many drinks are in the kitchen
JSON: {{"can_do": true, "reason": "need to navigate and count objects", "selected_skills": ["go_to_location", "count_objects"]}}

Command: tell me the name of the person in the bedroom
JSON: {{"can_do": true, "reason": "need to navigate and get info", "selected_skills": ["go_to_location", "get_person_info"]}}

Command: what is the biggest object on the table
JSON: {{"can_do": true, "reason": "need to navigate and find by property", "selected_skills": ["go_to_location", "find_object_by_property"]}}

Command: follow the person until they stop
JSON: {{"can_do": true, "reason": "follow skill", "selected_skills": ["follow_person"]}}

Command: follow morgan to the exit
JSON: {{"can_do": true, "reason": "follow skill", "selected_skills": ["follow_person"]}}

Command: say hello
JSON: {{"can_do": true, "reason": "just speak", "selected_skills": ["say"]}}

Command: what is your team affiliation
JSON: {{"can_do": true, "reason": "answer with known info", "selected_skills": ["say"]}}

Command: what day is it today
JSON: {{"can_do": true, "reason": "answer with known info", "selected_skills": ["say"]}}

Command: introduce yourself
JSON: {{"can_do": true, "reason": "just speak", "selected_skills": ["say"]}}

Command: make me a sandwich
JSON: {{"can_do": false, "reason": "no skill for cooking", "selected_skills": []}}

Command: {command}
JSON: """

SKILL_REFINER_PROMPT = """You are a robot skill checker.
You receive a list of skills chosen for a command and must fix missing dependencies.
Output ONLY JSON. No explanation.

DEPENDENCY RULES (apply all):
1. pick_up requires find_object before it — if find_object is missing, add it.
2. place_object requires pick_up before it — if pick_up is missing, add it.
3. give_to_person requires pick_up before it — if pick_up is missing, add it.
4. find_object, find_person, count_objects, count_people, get_person_info, find_object_by_property require go_to_location before them — if go_to_location is missing, add it.
5. guide_person and follow_person do NOT require go_to_location — never add it for those alone.
6. Output skills in correct execution order.
7. Never remove a skill from the input list — only add missing dependencies.
8. Keep skill names exactly as given (bare names, no arguments).

Output schema:
{{
  "refined_skills": ["skill_name", ...]
}}

EXAMPLES:

Command: pick up the apple
Input skills: ["pick_up"]
JSON: {{"refined_skills": ["go_to_location", "find_object", "pick_up"]}}

Command: bring the cola from the kitchen to the bedroom
Input skills: ["go_to_location", "pick_up", "place_object"]
JSON: {{"refined_skills": ["go_to_location", "find_object", "pick_up", "go_to_location", "place_object"]}}

Command: count the apples in the kitchen
Input skills: ["count_objects"]
JSON: {{"refined_skills": ["go_to_location", "count_objects"]}}

Command: how many people are waving in the living room
Input skills: ["count_people"]
JSON: {{"refined_skills": ["go_to_location", "count_people"]}}

Command: tell me the name of the person in the bedroom
Input skills: ["get_person_info"]
JSON: {{"refined_skills": ["go_to_location", "get_person_info"]}}

Command: find the apple in the kitchen and bring it to charlie
Input skills: ["go_to_location", "find_object", "pick_up", "give_to_person"]
JSON: {{"refined_skills": ["go_to_location", "find_object", "pick_up", "give_to_person"]}}

Command: give the cola to robin
Input skills: ["go_to_location", "find_person", "pick_up", "give_to_person"]
JSON: {{"refined_skills": ["go_to_location", "find_object", "pick_up", "find_person", "give_to_person"]}}

Command: guide charlie from the kitchen to the living room
Input skills: ["guide_person"]
JSON: {{"refined_skills": ["guide_person"]}}

Command: escort robin from the bedroom to the office
Input skills: ["guide_person"]
JSON: {{"refined_skills": ["guide_person"]}}

Command: say hello
Input skills: ["say"]
JSON: {{"refined_skills": ["say"]}}

Command: {command}
Input skills: {selected_skills}
JSON: """

PLANNER_PROMPT = """You are a robot planner. Output ONE JSON plan using only the given skills.
General knowledge: {general_knowledge}
Known locations: {locations}
Known objects: {objects}
Known people: {people}

RULES:
- Known locations are ONLY: {locations}. Furniture, appliances, and fixtures (sofa, bathroom, waste basket, refrigerator, coatrack, garage, sink, shelf, etc.) are NOT valid locations.
- FIRST check every room/place mentioned in the command. If ANY of them is NOT in known locations: output ONLY a single say step refusing. Do not plan any other steps.
- Only after confirming all locations are known: use ALL selected skills in the plan.
- find_object and find_person can search for ANY object/person, even if not in the known lists — do NOT refuse for unknown objects when find_object is selected.
- Fill args from the command and known world.
- say text contains only the spoken words.

Skills available for this command:
{selected_skill_lines}

EXAMPLES:
Command: go to the bathroom | Skills: go_to_location | Known locations: bedroom, kitchen, living room, office
Plan: {{"plan_description": "bathroom unknown", "steps": [{{"skill": "say", "args": {{"text": "I'm sorry, the bathroom is not on my map."}}}}]}}

Command: locate a snack in the bathroom | Skills: go_to_location, find_object | Known locations: bedroom, kitchen, living room, office
Plan: {{"plan_description": "bathroom unknown", "steps": [{{"skill": "say", "args": {{"text": "I'm sorry, the bathroom is not on my map."}}}}]}}

Command: put the pringles on the sofa | Skills: go_to_location, find_object, pick_up, place_object | Known locations: bedroom, kitchen, living room, office
Plan: {{"plan_description": "sofa is not a known location", "steps": [{{"skill": "say", "args": {{"text": "I'm sorry, the sofa is not a place I can navigate to."}}}}]}}

Command: fetch the apple and place it in the bathroom | Skills: go_to_location, find_object, pick_up, place_object | Known locations: bedroom, kitchen, living room, office
Plan: {{"plan_description": "bathroom is not a known location", "steps": [{{"skill": "say", "args": {{"text": "I'm sorry, the bathroom is not on my map."}}}}]}}

Command: fetch the pringles and put them on the sofa | Skills: go_to_location, find_object, pick_up, place_object | Known locations: bedroom, kitchen, living room, office
Plan: {{"plan_description": "sofa is not a known location", "steps": [{{"skill": "say", "args": {{"text": "I'm sorry, the sofa is not a place I can navigate to."}}}}]}}

Command: go to the waste basket then take the sponge and put it on the refrigerator | Skills: go_to_location, find_object, pick_up, place_object | Known locations: bedroom, kitchen, living room, office
Plan: {{"plan_description": "waste basket and refrigerator are not known locations", "steps": [{{"skill": "say", "args": {{"text": "I'm sorry, waste basket and refrigerator are not on my map."}}}}]}}

Command: lead Simone from the coatrack to the bathroom | Skills: guide_person | Known locations: bedroom, kitchen, living room, office
Plan: {{"plan_description": "coatrack and bathroom are not known locations", "steps": [{{"skill": "say", "args": {{"text": "I'm sorry, the coatrack and bathroom are not on my map."}}}}]}}

Command: find a pizza in the kitchen | Skills: go_to_location, find_object | Known locations: bedroom, kitchen, living room, office
Plan: {{"plan_description": "go to kitchen and search for pizza", "steps": [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}, {{"skill": "find_object", "args": {{"object": "pizza", "location": "kitchen"}}}}]}}

Command: find the cola in the kitchen | Skills: go_to_location, find_object | Known locations: bedroom, kitchen, living room, office
Plan: {{"plan_description": "go to kitchen and find cola", "steps": [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}, {{"skill": "find_object", "args": {{"object": "cola", "location": "kitchen"}}}}]}}

Command: count the apples in the office | Skills: go_to_location, count_objects | Known locations: bedroom, kitchen, living room, office
Plan: {{"plan_description": "go to office and count apples", "steps": [{{"skill": "go_to_location", "args": {{"location": "office"}}}}, {{"skill": "count_objects", "args": {{"object": "apple", "location": "office"}}}}]}}

Command: how many people waving in the living room | Skills: go_to_location, count_people | Known locations: bedroom, kitchen, living room, office
Plan: {{"plan_description": "go to living room and count people", "steps": [{{"skill": "go_to_location", "args": {{"location": "living room"}}}}, {{"skill": "count_people", "args": {{"gesture": "waving", "location": "living room"}}}}]}}

Command: bring the cola from the kitchen to the bedroom | Skills: go_to_location, find_object, pick_up, place_object | Known locations: bedroom, kitchen, living room, office
Plan: {{"plan_description": "fetch cola and bring to bedroom", "steps": [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}, {{"skill": "find_object", "args": {{"object": "cola", "location": "kitchen"}}}}, {{"skill": "pick_up", "args": {{"object": "cola"}}}}, {{"skill": "go_to_location", "args": {{"location": "bedroom"}}}}, {{"skill": "place_object", "args": {{"location": "bedroom"}}}}]}}

Command: meet Jane in the kitchen and escort her to the bedroom | Skills: go_to_location, find_person, guide_person | Known locations: bedroom, kitchen, living room, office
Plan: {{"plan_description": "find Jane in kitchen then escort to bedroom", "steps": [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}, {{"skill": "find_person", "args": {{"name": "jane", "location": "kitchen"}}}}, {{"skill": "guide_person", "args": {{"name": "jane", "start": "kitchen", "end": "bedroom"}}}}]}}

Command: find a person in the kitchen and say hi | Skills: go_to_location, find_person, say | Known locations: bedroom, kitchen, living room, office
Plan: {{"plan_description": "go to kitchen find person and say hi", "steps": [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}, {{"skill": "find_person", "args": {{"location": "kitchen"}}}}, {{"skill": "say", "args": {{"text": "Hi!"}}}}]}}

Command: guide charlie from kitchen to living room | Skills: guide_person | Known locations: bedroom, kitchen, living room, office
Plan: {{"plan_description": "guide charlie to living room", "steps": [{{"skill": "guide_person", "args": {{"name": "charlie", "start": "kitchen", "end": "living room"}}}}]}}

Command: {command} | Skills: {selected_skill_names} | Known locations: {locations}
Plan: """


def parse_json(raw: str) -> dict:
    start = raw.find("{")
    end = raw.rfind("}") + 1
    if start == -1 or end == 0:
        return {}
    return json.loads(raw[start:end])
