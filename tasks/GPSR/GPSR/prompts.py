import json

TRANSCRIPTION_CLEANER_PROMPT = """You are a transcription filter for a robot at a RoboCup@Home competition.
The speech-to-text system sometimes captures ambient speech before the actual command is given (people talking, announcements, the operator saying "the robot is ready now", etc.).
Your job is to find where the command starts and return only the command — nothing before it.
Output ONLY JSON. No explanation.

A command always starts with an imperative verb or a question word directed at the robot:
- Action verbs: go, navigate, find, locate, bring, get, fetch, take, put, place, deliver, count, tell, say, follow, escort, guide, lead, answer, introduce, greet, meet, give, pick up, carry, move, look for, search, come, return
- Question words starting a question to the robot: what, how many, where, who, which

RULES:
- Find the FIRST word that starts a valid robot command (imperative verb or question word as above).
- Return everything from that word to the end of the transcription, unchanged.
- Do NOT modify, correct, rephrase, or remove any words inside the command itself — copy them exactly.
- Do NOT add any words that were not in the original transcription.
- If the entire transcription is already a clean command (starts with an imperative or question), return it unchanged.
- If no command can be found, return the original transcription unchanged.

Output schema:
{{"cleaned": "<command text only>"}}

EXAMPLES:
Input: "oh the robot is ready now. Go to the tv stand then locate a drink and bring it to me"
JSON: {{"cleaned": "Go to the tv stand then locate a drink and bring it to me"}}

Input: "ok everyone the robot will now take instructions. bring me the apple from the dinner table"
JSON: {{"cleaned": "bring me the apple from the dinner table"}}

Input: "alright it seems to be on. find charlie in the bedroom and tell him your team affiliation"
JSON: {{"cleaned": "find charlie in the bedroom and tell him your team affiliation"}}

Input: "bring me the coke from the cabinet"
JSON: {{"cleaned": "bring me the coke from the cabinet"}}

Input: "what is the heaviest snack on the shelf"
JSON: {{"cleaned": "what is the heaviest snack on the shelf"}}

Input: {transcription}
JSON: """


# TODO: add memory to the examples
# TODO: Reduce the number of examples to the most important ones now with the local LLM is the good solution they are good when we provide a lot of examples but
# with memory and in case of CLOUD LLM we can reduce the number of examples. a test should be done to check the performance reducing the number of examples with gemma3.

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
- selected_skills contains ONLY the skill names (e.g. "find_object", "go_to_location"), never arguments or signatures (never "find_object(apple, kitchen)").
- can_do=false ONLY when NO skill matches the kind of action (e.g. cooking, flying).
- pick_up and place_object are ONLY for inanimate objects. If the command explicitly asks to physically pick up or place a PERSON as if they were an object (e.g. "pick up the person", "put the person on the sofa"), set can_do=false. Moving, escorting, transporting, or taking a person to a location is guide_person — never can_do=false.
- Missing locations/objects/people are NOT a reason for can_do=false — the planner handles those and manages the different cases.
- Any question, greeting, or request for information: can_do=true, selected_skills=["say"].
- If the result of a skill must be reported back to the operator (count, name, description, property), always include "say" in selected_skills.
- If the command involves bringing/fetching/delivering an object TO a person (including "me", or a named person), use "give_to_person" as the final delivery skill, NOT "place_object". Use "place_object" only when placing at a location with no person recipient.
- get_person_info is ONLY for learning information ABOUT a person (their name, age, pose, gesture). If the command is about saying or reporting something TO a person, use find_person + say instead — never get_person_info.
- Verb synonyms — map these to the correct skill:
  - find_person: "meet", "locate" (a person), "look for" (a person), "contact", "get to know", "get acquainted with"
  - find_object: "locate" (an object), "look for" (an object), "fetch", "grasp"
  - pick_up: "take", "get", "grasp", "fetch", "pick up"
  - guide_person: "escort", "lead", "accompany", "take" (a person somewhere)
  - follow_person: "follow", "accompany" (behind someone)
  - say: "tell", "talk", "greet", "salute", "say hello to", "introduce yourself to", "answer", "describe"

EXAMPLES:
Command: bring me the apple from the dinner table
JSON: {{"can_do": true, "reason": "fetch object and deliver to operator", "selected_skills": ["go_to_location", "find_object", "pick_up", "give_to_person"]}}

Command: get the coke and give it to charlie
JSON: {{"can_do": true, "reason": "fetch object and give to named person", "selected_skills": ["go_to_location", "find_object", "pick_up", "find_person", "give_to_person"]}}

Command: take the pringles from the shelf and put them on the coffee table
JSON: {{"can_do": true, "reason": "fetch object and place at a location", "selected_skills": ["go_to_location", "find_object", "pick_up", "place_object"]}}

Command: take the sponge from the laundry table and throw it in the trash
JSON: {{"can_do": true, "reason": "fetch object and place in trash bin", "selected_skills": ["go_to_location", "find_object", "pick_up", "place_object"]}}

Command: find the pepsi
JSON: {{"can_do": true, "reason": "navigate, search for object, report back", "selected_skills": ["go_to_location", "find_object", "say"]}}

Command: count how many drinks are on the cabinet
JSON: {{"can_do": true, "reason": "navigate, count objects at sub-location, report back", "selected_skills": ["go_to_location", "count_objects", "say"]}}

Command: how many snacks are on the shelf
JSON: {{"can_do": true, "reason": "navigate, count objects, report back", "selected_skills": ["go_to_location", "count_objects", "say"]}}

Command: what is the biggest fruit on the dinner table
JSON: {{"can_do": true, "reason": "navigate, find object by property, report back", "selected_skills": ["go_to_location", "find_object_by_property", "say"]}}

Command: tell me the lightest snack on the shelf
JSON: {{"can_do": true, "reason": "navigate, find object by comparative property, report back", "selected_skills": ["go_to_location", "find_object_by_property", "say"]}}

Command: tell me how many people are in the living room
JSON: {{"can_do": true, "reason": "navigate, count people, report back", "selected_skills": ["go_to_location", "count_people", "say"]}}

Command: how many sitting people are in the bedroom
JSON: {{"can_do": true, "reason": "navigate, count people by pose, report back", "selected_skills": ["go_to_location", "count_people", "say"]}}

Command: count people wearing blue shirts in the kitchen
JSON: {{"can_do": true, "reason": "navigate, count people by clothing, report back", "selected_skills": ["go_to_location", "count_people", "say"]}}

Command: tell me the name of the person in the laundry
JSON: {{"can_do": true, "reason": "navigate, get person info, report back", "selected_skills": ["go_to_location", "get_person_info", "say"]}}

Command: what is the pose of the person at the sofa
JSON: {{"can_do": true, "reason": "navigate, get person info, report back", "selected_skills": ["go_to_location", "get_person_info", "say"]}}

Command: describe the person standing in the kitchen
JSON: {{"can_do": true, "reason": "navigate, find person, get visual info to describe", "selected_skills": ["go_to_location", "find_person", "get_person_info", "say"]}}

Command: tell the gesture of the person at the bed to the person at the sofa
JSON: {{"can_do": true, "reason": "get info about one person and report it to another", "selected_skills": ["go_to_location", "get_person_info", "find_person", "say"]}}

Command: find charlie in the bedroom and tell him your team name
JSON: {{"can_do": true, "reason": "navigate to person and say information to them", "selected_skills": ["go_to_location", "find_person", "say"]}}

Command: meet simone in the living room and say hello
JSON: {{"can_do": true, "reason": "meet=find_person then greet", "selected_skills": ["go_to_location", "find_person", "say"]}}

Command: greet the person wearing a black shirt in the bedroom and follow them
JSON: {{"can_do": true, "reason": "find person by clothing, greet, then follow", "selected_skills": ["go_to_location", "find_person", "say", "follow_person"]}}

Command: say something about yourself to the waving person in the living room
JSON: {{"can_do": true, "reason": "find person by gesture then say info", "selected_skills": ["go_to_location", "find_person", "say"]}}

Command: get acquainted with the person pointing to the right in the kitchen
JSON: {{"can_do": true, "reason": "find person by gesture then interact", "selected_skills": ["go_to_location", "find_person", "say"]}}

Command: head over to the laundry and locate morgan
JSON: {{"can_do": true, "reason": "navigate then find person by name", "selected_skills": ["go_to_location", "find_person"]}}

Command: meet simone at the sofa then find her in the bedroom
JSON: {{"can_do": true, "reason": "meet at one location then find in another room", "selected_skills": ["go_to_location", "find_person"]}}

Command: meet jane at the dinner table and escort her to the bedroom
JSON: {{"can_do": true, "reason": "find person at sub-location then guide to room", "selected_skills": ["go_to_location", "find_person", "guide_person"]}}

Command: escort the waving person from the living room to the kitchen
JSON: {{"can_do": true, "reason": "find person by gesture then guide between rooms", "selected_skills": ["go_to_location", "find_person", "guide_person"]}}

Command: guide the person wearing a blue shirt from the entrance to the bedroom
JSON: {{"can_do": true, "reason": "find person by clothing then guide to destination", "selected_skills": ["go_to_location", "find_person", "guide_person"]}}

Command: lead the person sitting at the sofa to the kitchen
JSON: {{"can_do": true, "reason": "lead=guide_person, find by pose first", "selected_skills": ["go_to_location", "find_person", "guide_person"]}}

Command: follow the standing person in the laundry
JSON: {{"can_do": true, "reason": "find person by pose then follow", "selected_skills": ["go_to_location", "find_person", "follow_person"]}}

Command: follow the person raising their left arm at the entrance
JSON: {{"can_do": true, "reason": "find person by gesture at location then follow", "selected_skills": ["go_to_location", "find_person", "follow_person"]}}

Command: follow morgan to the exit
JSON: {{"can_do": true, "reason": "find person by name then follow", "selected_skills": ["find_person", "follow_person"]}}

Command: say hello
JSON: {{"can_do": true, "reason": "just speak", "selected_skills": ["say"]}}

Command: what is your team affiliation
JSON: {{"can_do": true, "reason": "answer with known info", "selected_skills": ["say"]}}

Command: what day is it today
JSON: {{"can_do": true, "reason": "answer with known info", "selected_skills": ["say"]}}

Command: what day is tomorrow
JSON: {{"can_do": true, "reason": "answer with known info", "selected_skills": ["say"]}}

Command: introduce yourself
JSON: {{"can_do": true, "reason": "just speak", "selected_skills": ["say"]}}

Command: answer the door
JSON: {{"can_do": false, "reason": "no skill for answering a door", "selected_skills": []}}

Command: make me a sandwich
JSON: {{"can_do": false, "reason": "no skill for cooking on the available skills", "selected_skills": []}}

Command: pick up the person from the sofa and bring them to the kitchen
JSON: {{"can_do": false, "reason": "pick_up cannot be used on a person — use guide_person to escort people between locations", "selected_skills": []}}

Command: navigate to the kitchen find the lying person and export them to the dishwasher
JSON: {{"can_do": true, "reason": "export=guide_person, escort person from kitchen to dishwasher", "selected_skills": ["go_to_location", "find_person", "guide_person"]}}

Command: {command}
JSON: """

# To add some rules on the plan a pick up action should be followed by a go to location and a place object action. a give to person action should be followed by a go to location action.
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
9. Any plan that picks up an object must end with either give_to_person (if delivering to a person) or place_object (if placing at a location) — never leave the object undelivered.  

Output schema:
{{
  "refined_skills": ["skill_name", ...]
}}

EXAMPLES:

Command: 
Input skills: ["pick_up"]
JSON: {{"refined_skills": ["go_to_location", "find_object", "pick_up", "go_to_location", "give_to_person"]}}


Command: {command}
Input skills: {selected_skills}
JSON: """

PLANNER_PROMPT = """You are a robot planner. Output ONE JSON plan using only the given skills.
General knowledge: {general_knowledge}
Known locations: {locations}
Placeable locations (place_object allowed here): {placement_locations}
Known objects: {objects}
Known people: {people}

RULES:
- Known locations are ONLY: {locations}. Any place not in this list is unknown — the robot cannot navigate there.
- FIRST check every room/place mentioned in the command. If ANY of them is NOT in known locations: output ONLY a single say step refusing. Do not plan any other steps.
- Only after confirming all locations are known: use ALL selected skills in the plan.
- place_object MUST only target a placeable location (from: {placement_locations}). Never place objects at navigation-only locations such as entrance, exit, sink, refrigerator, kitchen trash bin, or coat rack.
- Every object is in the known objects list with format "name (category at sub-location in room)". When navigating to find an object, ALWAYS go to the room first, then the sub-location: go_to_location(room) → go_to_location(sub-location) → find_object.
- Fill args from the command and known world.
- say text contains only the spoken words.
- ALWAYS complete the task end-to-end. If the robot gathers information (count, name, description, property) it MUST return to the instruction point and report the result with a say step. Never leave the result unreported.
- If the plan is a single say step (e.g. answering a question), the text MUST be a direct factual answer using ONLY information from General knowledge above. Do not invent facts. The answer must be specific and complete — never vague or generic. If the answer cannot be found in the general knowledge, output a say step saying "I'm sorry, I don't have that information."
- If the command involves bringing/fetching/delivering an object TO a person (including "me", "the operator", or a named person), the final step MUST be give_to_person, NOT place_object. place_object is only for placing objects at a location with no person recipient.
- get_person_info is ONLY for learning information ABOUT a person. If the command is about saying something TO a person, use find_person then say — never get_person_info.

Skills available for this command:
{selected_skill_lines}

EXAMPLES:
Command: go to the bathroom | Skills: go_to_location | Known locations: bedroom, kitchen, laundry, living room
Plan: {{"plan_description": "bathroom is not a known location", "steps": [{{"skill": "say", "args": {{"text": "I'm sorry, the bathroom is not on my map."}}}}]}}

Command: find the apple in the garage | Skills: go_to_location, find_object | Known locations: bedroom, kitchen, laundry, living room, dinner table
Plan: {{"plan_description": "garage is not a known location", "steps": [{{"skill": "say", "args": {{"text": "I'm sorry, the garage is not on my map."}}}}]}}

Command: put the pringles on the sink | Skills: go_to_location, find_object, pick_up, place_object | Known locations: bedroom, kitchen, laundry, living room, shelf, sink | Objects: pringles (snack at shelf in laundry) | Placeable locations: laundry table, shelf, bed, tv stand, sofa, coffee table, cabinet, counter, cooking table, dishwasher, dinner table, bedside table
Plan: {{"plan_description": "sink is not a placeable location", "steps": [{{"skill": "say", "args": {{"text": "I'm sorry, the sink is not a valid location to place objects."}}}}]}}

Command: lead robin from the entrance to the garden | Skills: go_to_location, find_person, guide_person | Known locations: bedroom, kitchen, laundry, living room, entrance
Plan: {{"plan_description": "garden is not a known location", "steps": [{{"skill": "say", "args": {{"text": "I'm sorry, the garden is not on my map."}}}}]}}

Command: go to the living room | Skills: go_to_location | Known locations: bedroom, kitchen, laundry, living room
Plan: {{"plan_description": "navigate to living room", "steps": [{{"skill": "go_to_location", "args": {{"location": "living room"}}}}]}}

Command: find the coke | Skills: go_to_location, find_object, say | Known locations: bedroom, kitchen, laundry, living room, cabinet | Objects: coke (drink at cabinet in kitchen)
Plan: {{"plan_description": "go to kitchen then cabinet, find coke, report back", "steps": [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}, {{"skill": "go_to_location", "args": {{"location": "cabinet"}}}}, {{"skill": "find_object", "args": {{"object": "coke", "location": "cabinet"}}}}, {{"skill": "go_to_location", "args": {{"location": "instruction point"}}}}, {{"skill": "say", "args": {{"text": "I found the coke on the cabinet."}}}}]}}

Command: bring me the apple | Skills: go_to_location, find_object, pick_up, give_to_person | Known locations: bedroom, kitchen, laundry, living room, dinner table | Objects: apple (fruit at dinner table in kitchen)
Plan: {{"plan_description": "fetch apple from dinner table and give to operator", "steps": [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}, {{"skill": "go_to_location", "args": {{"location": "dinner table"}}}}, {{"skill": "find_object", "args": {{"object": "apple", "location": "dinner table"}}}}, {{"skill": "pick_up", "args": {{"object": "apple"}}}}, {{"skill": "go_to_location", "args": {{"location": "instruction point"}}}}, {{"skill": "give_to_person", "args": {{"person": "operator"}}}}]}}

Command: get the pepsi and give it to morgan | Skills: go_to_location, find_object, pick_up, find_person, give_to_person | Known locations: bedroom, kitchen, laundry, living room, cabinet | Objects: pepsi (drink at cabinet in kitchen)
Plan: {{"plan_description": "fetch pepsi from cabinet, find morgan, give to morgan", "steps": [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}, {{"skill": "go_to_location", "args": {{"location": "cabinet"}}}}, {{"skill": "find_object", "args": {{"object": "pepsi", "location": "cabinet"}}}}, {{"skill": "pick_up", "args": {{"object": "pepsi"}}}}, {{"skill": "find_person", "args": {{"name": "morgan"}}}}, {{"skill": "give_to_person", "args": {{"person": "morgan"}}}}]}}

Command: take the pringles and put them on the coffee table | Skills: go_to_location, find_object, pick_up, place_object | Known locations: bedroom, kitchen, laundry, living room, shelf, coffee table | Objects: pringles (snack at shelf in laundry)
Plan: {{"plan_description": "fetch pringles from shelf in laundry and place on coffee table in living room", "steps": [{{"skill": "go_to_location", "args": {{"location": "laundry"}}}}, {{"skill": "go_to_location", "args": {{"location": "shelf"}}}}, {{"skill": "find_object", "args": {{"object": "pringles", "location": "shelf"}}}}, {{"skill": "pick_up", "args": {{"object": "pringles"}}}}, {{"skill": "go_to_location", "args": {{"location": "living room"}}}}, {{"skill": "go_to_location", "args": {{"location": "coffee table"}}}}, {{"skill": "place_object", "args": {{"location": "coffee table"}}}}]}}

Command: take the sponge from the laundry table and throw it in the trash | Skills: go_to_location, find_object, pick_up, place_object | Known locations: bedroom, kitchen, laundry, living room, laundry table, laundry trash bin | Objects: sponge (cleaning supply at laundry table in laundry)
Plan: {{"plan_description": "pick up sponge from laundry table and place in laundry trash bin", "steps": [{{"skill": "go_to_location", "args": {{"location": "laundry"}}}}, {{"skill": "go_to_location", "args": {{"location": "laundry table"}}}}, {{"skill": "find_object", "args": {{"object": "sponge", "location": "laundry table"}}}}, {{"skill": "pick_up", "args": {{"object": "sponge"}}}}, {{"skill": "go_to_location", "args": {{"location": "laundry trash bin"}}}}, {{"skill": "place_object", "args": {{"location": "laundry trash bin"}}}}]}}

Command: count how many drinks are on the cabinet | Skills: go_to_location, count_objects, say | Known locations: bedroom, kitchen, laundry, living room, cabinet
Plan: {{"plan_description": "go to kitchen then cabinet, count drinks, report back", "steps": [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}, {{"skill": "go_to_location", "args": {{"location": "cabinet"}}}}, {{"skill": "count_objects", "args": {{"object": "drink", "location": "cabinet"}}}}, {{"skill": "go_to_location", "args": {{"location": "instruction point"}}}}, {{"skill": "say", "args": {{"text": "There are [result] drinks on the cabinet."}}}}]}}

Command: how many snacks are on the shelf | Skills: go_to_location, count_objects, say | Known locations: bedroom, kitchen, laundry, living room, shelf
Plan: {{"plan_description": "go to laundry then shelf, count snacks, report back", "steps": [{{"skill": "go_to_location", "args": {{"location": "laundry"}}}}, {{"skill": "go_to_location", "args": {{"location": "shelf"}}}}, {{"skill": "count_objects", "args": {{"object": "snack", "location": "shelf"}}}}, {{"skill": "go_to_location", "args": {{"location": "instruction point"}}}}, {{"skill": "say", "args": {{"text": "There are [result] snacks on the shelf."}}}}]}}

Command: what is the biggest fruit on the dinner table | Skills: go_to_location, find_object_by_property, say | Known locations: bedroom, kitchen, laundry, living room, dinner table
Plan: {{"plan_description": "go to kitchen then dinner table, find biggest fruit, report back", "steps": [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}, {{"skill": "go_to_location", "args": {{"location": "dinner table"}}}}, {{"skill": "find_object_by_property", "args": {{"property": "biggest", "object": "fruit", "location": "dinner table"}}}}, {{"skill": "go_to_location", "args": {{"location": "instruction point"}}}}, {{"skill": "say", "args": {{"text": "The biggest fruit on the dinner table is [result]."}}}}]}}

Command: tell me what is the lightest snack on the shelf | Skills: go_to_location, find_object_by_property, say | Known locations: bedroom, kitchen, laundry, living room, shelf
Plan: {{"plan_description": "go to laundry then shelf, find lightest snack, report back", "steps": [{{"skill": "go_to_location", "args": {{"location": "laundry"}}}}, {{"skill": "go_to_location", "args": {{"location": "shelf"}}}}, {{"skill": "find_object_by_property", "args": {{"property": "lightest", "object": "snack", "location": "shelf"}}}}, {{"skill": "go_to_location", "args": {{"location": "instruction point"}}}}, {{"skill": "say", "args": {{"text": "The lightest snack on the shelf is [result]."}}}}]}}

Command: tell me how many people are in the living room | Skills: go_to_location, count_people, say | Known locations: bedroom, kitchen, laundry, living room
Plan: {{"plan_description": "go to living room, count people, return and report", "steps": [{{"skill": "go_to_location", "args": {{"location": "living room"}}}}, {{"skill": "count_people", "args": {{"location": "living room"}}}}, {{"skill": "go_to_location", "args": {{"location": "instruction point"}}}}, {{"skill": "say", "args": {{"text": "There are [result] people in the living room."}}}}]}}

Command: how many sitting people are in the bedroom | Skills: go_to_location, count_people, say | Known locations: bedroom, kitchen, laundry, living room
Plan: {{"plan_description": "go to bedroom, count sitting people, report back", "steps": [{{"skill": "go_to_location", "args": {{"location": "bedroom"}}}}, {{"skill": "count_people", "args": {{"pose": "sitting", "location": "bedroom"}}}}, {{"skill": "go_to_location", "args": {{"location": "instruction point"}}}}, {{"skill": "say", "args": {{"text": "There are [result] sitting people in the bedroom."}}}}]}}

Command: how many people waving in the laundry | Skills: go_to_location, count_people, say | Known locations: bedroom, kitchen, laundry, living room
Plan: {{"plan_description": "go to laundry, count waving people, report back", "steps": [{{"skill": "go_to_location", "args": {{"location": "laundry"}}}}, {{"skill": "count_people", "args": {{"gesture": "waving", "location": "laundry"}}}}, {{"skill": "go_to_location", "args": {{"location": "instruction point"}}}}, {{"skill": "say", "args": {{"text": "There are [result] waving people in the laundry."}}}}]}}

Command: count people wearing blue shirts in the kitchen | Skills: go_to_location, count_people, say | Known locations: bedroom, kitchen, laundry, living room
Plan: {{"plan_description": "go to kitchen, count people wearing blue shirts, report back", "steps": [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}, {{"skill": "count_people", "args": {{"clothes": "blue shirt", "location": "kitchen"}}}}, {{"skill": "go_to_location", "args": {{"location": "instruction point"}}}}, {{"skill": "say", "args": {{"text": "There are [result] people wearing blue shirts in the kitchen."}}}}]}}

Command: tell me the name of the person in the laundry | Skills: go_to_location, get_person_info, say | Known locations: bedroom, kitchen, laundry, living room
Plan: {{"plan_description": "go to laundry, get person name, return and report", "steps": [{{"skill": "go_to_location", "args": {{"location": "laundry"}}}}, {{"skill": "get_person_info", "args": {{"info": "name", "location": "laundry"}}}}, {{"skill": "go_to_location", "args": {{"location": "instruction point"}}}}, {{"skill": "say", "args": {{"text": "The name of the person in the laundry is [result]."}}}}]}}

Command: what is the pose of the person at the sofa | Skills: go_to_location, get_person_info, say | Known locations: bedroom, kitchen, laundry, living room, sofa
Plan: {{"plan_description": "go to living room then sofa, get person pose, report back", "steps": [{{"skill": "go_to_location", "args": {{"location": "living room"}}}}, {{"skill": "go_to_location", "args": {{"location": "sofa"}}}}, {{"skill": "get_person_info", "args": {{"info": "pose", "location": "sofa"}}}}, {{"skill": "go_to_location", "args": {{"location": "instruction point"}}}}, {{"skill": "say", "args": {{"text": "The pose of the person at the sofa is [result]."}}}}]}}

Command: tell the gesture of the person at the bed to the person at the sofa | Skills: go_to_location, get_person_info, find_person, say | Known locations: bedroom, kitchen, laundry, living room, bed, sofa
Plan: {{"plan_description": "get gesture of person at bed then report to person at sofa", "steps": [{{"skill": "go_to_location", "args": {{"location": "bedroom"}}}}, {{"skill": "go_to_location", "args": {{"location": "bed"}}}}, {{"skill": "get_person_info", "args": {{"info": "gesture", "location": "bed"}}}}, {{"skill": "go_to_location", "args": {{"location": "living room"}}}}, {{"skill": "go_to_location", "args": {{"location": "sofa"}}}}, {{"skill": "find_person", "args": {{"location": "sofa"}}}}, {{"skill": "say", "args": {{"text": "The gesture of the person at the bed is [result]."}}}}]}}

Command: find charlie in the bedroom and tell him your teams affiliation | Skills: go_to_location, find_person, say | Known locations: bedroom, kitchen, laundry, living room
Plan: {{"plan_description": "go to bedroom, find charlie, tell him team affiliation", "steps": [{{"skill": "go_to_location", "args": {{"location": "bedroom"}}}}, {{"skill": "find_person", "args": {{"name": "charlie", "location": "bedroom"}}}}, {{"skill": "say", "args": {{"text": "My team is LASR, affiliated with King's College London."}}}}]}}

Command: find a person in the kitchen and say hi | Skills: go_to_location, find_person, say | Known locations: bedroom, kitchen, laundry, living room
Plan: {{"plan_description": "go to kitchen, find person, say hi", "steps": [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}, {{"skill": "find_person", "args": {{"location": "kitchen"}}}}, {{"skill": "say", "args": {{"text": "Hi!"}}}}]}}

Command: say something about yourself to the waving person in the living room | Skills: go_to_location, find_person, say | Known locations: bedroom, kitchen, laundry, living room
Plan: {{"plan_description": "go to living room, find waving person, say info about self", "steps": [{{"skill": "go_to_location", "args": {{"location": "living room"}}}}, {{"skill": "find_person", "args": {{"gesture": "waving", "location": "living room"}}}}, {{"skill": "say", "args": {{"text": "I am TIAGo, a robot from LASR at King's College London."}}}}]}}

Command: greet the person wearing a black shirt in the bedroom and follow them | Skills: go_to_location, find_person, say, follow_person | Known locations: bedroom, kitchen, laundry, living room
Plan: {{"plan_description": "go to bedroom, find person in black shirt, greet, then follow", "steps": [{{"skill": "go_to_location", "args": {{"location": "bedroom"}}}}, {{"skill": "find_person", "args": {{"clothes": "black shirt", "location": "bedroom"}}}}, {{"skill": "say", "args": {{"text": "Hello!"}}}}, {{"skill": "follow_person", "args": {{}}}}]}}

Command: meet simone at the sofa then find her in the bedroom | Skills: go_to_location, find_person | Known locations: bedroom, kitchen, laundry, living room, sofa
Plan: {{"plan_description": "go to living room then sofa to meet simone, then search for her in bedroom", "steps": [{{"skill": "go_to_location", "args": {{"location": "living room"}}}}, {{"skill": "go_to_location", "args": {{"location": "sofa"}}}}, {{"skill": "find_person", "args": {{"name": "simone", "location": "sofa"}}}}, {{"skill": "go_to_location", "args": {{"location": "bedroom"}}}}, {{"skill": "find_person", "args": {{"name": "simone", "location": "bedroom"}}}}]}}

Command: meet jane at the dinner table and escort her to the bedroom | Skills: go_to_location, find_person, guide_person | Known locations: bedroom, kitchen, laundry, living room, dinner table
Plan: {{"plan_description": "go to kitchen then dinner table, find jane, guide to bedroom", "steps": [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}, {{"skill": "go_to_location", "args": {{"location": "dinner table"}}}}, {{"skill": "find_person", "args": {{"name": "jane", "location": "dinner table"}}}}, {{"skill": "guide_person", "args": {{"name": "jane", "start": "dinner table", "end": "bedroom"}}}}]}}

Command: escort the waving person from the living room to the kitchen | Skills: go_to_location, find_person, guide_person | Known locations: bedroom, kitchen, laundry, living room
Plan: {{"plan_description": "go to living room, find waving person, guide to kitchen", "steps": [{{"skill": "go_to_location", "args": {{"location": "living room"}}}}, {{"skill": "find_person", "args": {{"gesture": "waving", "location": "living room"}}}}, {{"skill": "guide_person", "args": {{"start": "living room", "end": "kitchen"}}}}]}}

Command: guide the person wearing a blue shirt from the entrance to the bedroom | Skills: go_to_location, find_person, guide_person | Known locations: bedroom, kitchen, laundry, living room, entrance
Plan: {{"plan_description": "go to entrance, find person wearing blue shirt, guide to bedroom", "steps": [{{"skill": "go_to_location", "args": {{"location": "entrance"}}}}, {{"skill": "find_person", "args": {{"clothes": "blue shirt", "location": "entrance"}}}}, {{"skill": "guide_person", "args": {{"start": "entrance", "end": "bedroom"}}}}]}}

Command: follow the standing person in the laundry | Skills: go_to_location, find_person, follow_person | Known locations: bedroom, kitchen, laundry, living room
Plan: {{"plan_description": "go to laundry, find standing person, follow them", "steps": [{{"skill": "go_to_location", "args": {{"location": "laundry"}}}}, {{"skill": "find_person", "args": {{"pose": "standing", "location": "laundry"}}}}, {{"skill": "follow_person", "args": {{}}}}]}}

Command: follow the person raising their left arm at the entrance | Skills: go_to_location, find_person, follow_person | Known locations: bedroom, kitchen, laundry, living room, entrance
Plan: {{"plan_description": "go to entrance, find person raising left arm, follow them", "steps": [{{"skill": "go_to_location", "args": {{"location": "entrance"}}}}, {{"skill": "find_person", "args": {{"gesture": "raising their left arm", "location": "entrance"}}}}, {{"skill": "follow_person", "args": {{}}}}]}}

Command: follow morgan to the exit | Skills: find_person, follow_person | Known locations: bedroom, kitchen, laundry, living room, exit
Plan: {{"plan_description": "find morgan then follow them", "steps": [{{"skill": "find_person", "args": {{"name": "morgan"}}}}, {{"skill": "follow_person", "args": {{}}}}]}}

Command: what is your team affiliation | Skills: say | Known locations: bedroom, kitchen, laundry, living room
Plan: {{"plan_description": "answer question about team affiliation", "steps": [{{"skill": "say", "args": {{"text": "My team is LASR from the United Kingdom at King's College London."}}}}]}}

Command: what day is today | Skills: say | Known locations: bedroom, kitchen, laundry, living room
Plan: {{"plan_description": "answer question about today's date", "steps": [{{"skill": "say", "args": {{"text": "Today is [day of week], [day of month] [month] [year]."}}}}]}}

Command: {command} | Skills: {selected_skill_names} | Known locations: {locations} | Placeable locations: {placement_locations}
Plan: """

# JUST TO REPRAHSE THE PLAN IN A NATURAL WAY BEFORE EXECUTING IT
ANNOUNCE_PLAN_PROMPT = """You are a robot assistant.
Turn the plan below into ONE spoken announcement listing every step in order.
Start with "Here is my plan." then say Step 1, Step 2, ... Step N — one short phrase per step derived from the skill and args.
Use only words that will be spoken aloud. No bullet points or JSON in the announcement. The announcement should not execute the plan as provided in the examples.

PERSPECTIVE RULE: The operator gives commands in first person ("bring it to me", "tell me", "show me").
When describing what you will do, reframe these as second person: "bring it to you", "tell you", "show you".
Never say "bring it to me" or "tell me" in the announcement — always say "you" when referring to the operator.
When the robot is reporting or saying something about itself (its team, affiliation, name, etc.), use "my" — e.g. "report my affiliation", "say my team name".

User command: {command}
Plan summary: {plan_description}
Steps: {steps_json}

Output ONLY JSON:
{{"announcement": "<full spoken announcement>"}}

EXAMPLES:
Command: take the pringles from the shelf and put them on the coffee table
Plan summary: fetch pringles from shelf in laundry and place on coffee table
Steps: [{{"skill": "go_to_location", "args": {{"location": "laundry"}}}}, {{"skill": "go_to_location", "args": {{"location": "shelf"}}}}, {{"skill": "find_object", "args": {{"object": "pringles", "location": "shelf"}}}}, {{"skill": "pick_up", "args": {{"object": "pringles"}}}}, {{"skill": "go_to_location", "args": {{"location": "living room"}}}}, {{"skill": "go_to_location", "args": {{"location": "coffee table"}}}}, {{"skill": "place_object", "args": {{"location": "coffee table"}}}}]
JSON: {{"announcement": "Here is my plan. Step 1: go to the laundry. Step 2: go to the shelf. Step 3: find the pringles. Step 4: pick up the pringles. Step 5: go to the living room. Step 6: go to the coffee table. Step 7: place the pringles on the coffee table."}}

Command: bring me the apple
Plan summary: fetch apple from dinner table and give to operator
Steps: [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}, {{"skill": "go_to_location", "args": {{"location": "dinner table"}}}}, {{"skill": "find_object", "args": {{"object": "apple", "location": "dinner table"}}}}, {{"skill": "pick_up", "args": {{"object": "apple"}}}}, {{"skill": "go_to_location", "args": {{"location": "instruction point"}}}}, {{"skill": "give_to_person", "args": {{"person": "operator"}}}}]
JSON: {{"announcement": "Here is my plan. Step 1: go to the kitchen. Step 2: go to the dinner table. Step 3: find the apple. Step 4: pick up the apple. Step 5: return to you. Step 6: give you the apple."}}

Command: take the sponge from the laundry table and throw it in the trash
Plan summary: pick up sponge from laundry table and place in laundry trash bin
Steps: [{{"skill": "go_to_location", "args": {{"location": "laundry"}}}}, {{"skill": "go_to_location", "args": {{"location": "laundry table"}}}}, {{"skill": "find_object", "args": {{"object": "sponge", "location": "laundry table"}}}}, {{"skill": "pick_up", "args": {{"object": "sponge"}}}}, {{"skill": "go_to_location", "args": {{"location": "laundry trash bin"}}}}, {{"skill": "place_object", "args": {{"location": "laundry trash bin"}}}}]
JSON: {{"announcement": "Here is my plan. Step 1: go to the laundry. Step 2: go to the laundry table. Step 3: find the sponge. Step 4: pick up the sponge. Step 5: go to the laundry trash bin. Step 6: throw the sponge in the trash."}}

Command: get the pepsi and give it to morgan
Plan summary: fetch pepsi from cabinet, find morgan, give to morgan
Steps: [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}, {{"skill": "go_to_location", "args": {{"location": "cabinet"}}}}, {{"skill": "find_object", "args": {{"object": "pepsi", "location": "cabinet"}}}}, {{"skill": "pick_up", "args": {{"object": "pepsi"}}}}, {{"skill": "find_person", "args": {{"name": "morgan"}}}}, {{"skill": "give_to_person", "args": {{"person": "morgan"}}}}]
JSON: {{"announcement": "Here is my plan. Step 1: go to the kitchen. Step 2: go to the cabinet. Step 3: find the pepsi. Step 4: pick up the pepsi. Step 5: find morgan. Step 6: give the pepsi to morgan."}}

Command: tell me how many people are in the living room
Plan summary: count people in living room and report
Steps: [{{"skill": "go_to_location", "args": {{"location": "living room"}}}}, {{"skill": "count_people", "args": {{"location": "living room"}}}}, {{"skill": "go_to_location", "args": {{"location": "instruction point"}}}}, {{"skill": "say", "args": {{"text": "There are [result] people in the living room."}}}}]
JSON: {{"announcement": "Here is my plan. Step 1: go to the living room. Step 2: count the people. Step 3: return to you and report the count."}}

Command: how many sitting people are in the bedroom
Plan summary: go to bedroom, count sitting people, report back
Steps: [{{"skill": "go_to_location", "args": {{"location": "bedroom"}}}}, {{"skill": "count_people", "args": {{"pose": "sitting", "location": "bedroom"}}}}, {{"skill": "go_to_location", "args": {{"location": "instruction point"}}}}, {{"skill": "say", "args": {{"text": "There are [result] sitting people in the bedroom."}}}}]
JSON: {{"announcement": "Here is my plan. Step 1: go to the bedroom. Step 2: count the sitting people. Step 3: return to you and report the count."}}

Command: what is the biggest fruit on the dinner table
Plan summary: go to kitchen then dinner table, find biggest fruit, report back
Steps: [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}, {{"skill": "go_to_location", "args": {{"location": "dinner table"}}}}, {{"skill": "find_object_by_property", "args": {{"property": "biggest", "object": "fruit", "location": "dinner table"}}}}, {{"skill": "go_to_location", "args": {{"location": "instruction point"}}}}, {{"skill": "say", "args": {{"text": "The biggest fruit on the dinner table is [result]."}}}}]
JSON: {{"announcement": "Here is my plan. Step 1: go to the kitchen. Step 2: go to the dinner table. Step 3: find the biggest fruit. Step 4: return to you and report the result."}}

Command: meet jane at the dinner table and escort her to the bedroom
Plan summary: go to kitchen then dinner table, find jane, guide to bedroom
Steps: [{{"skill": "go_to_location", "args": {{"location": "kitchen"}}}}, {{"skill": "go_to_location", "args": {{"location": "dinner table"}}}}, {{"skill": "find_person", "args": {{"name": "jane", "location": "dinner table"}}}}, {{"skill": "guide_person", "args": {{"name": "jane", "start": "dinner table", "end": "bedroom"}}}}]
JSON: {{"announcement": "Here is my plan. Step 1: go to the kitchen. Step 2: go to the dinner table. Step 3: find jane. Step 4: escort jane to the bedroom."}}

Command: follow the standing person in the laundry
Plan summary: go to laundry, find standing person, follow them
Steps: [{{"skill": "go_to_location", "args": {{"location": "laundry"}}}}, {{"skill": "find_person", "args": {{"pose": "standing", "location": "laundry"}}}}, {{"skill": "follow_person", "args": {{}}}}]
JSON: {{"announcement": "Here is my plan. Step 1: go to the laundry. Step 2: find the standing person. Step 3: follow them."}}

Command: go to the living room
Plan summary: navigate to living room
Steps: [{{"skill": "go_to_location", "args": {{"location": "living room"}}}}]
JSON: {{"announcement": "Here is my plan. Step 1: go to the living room."}}

DO NOT SAY THE TEXT OF THE SKILL IN THE ANNOUNCEMENT WITH THE ARGUMENT FOR EXAMPLES SUCH AS THE FOLLOWING!!
Command: what is your teams affiliation
Plan summary: answer question about team affiliation
Steps: [{{"skill": "say", "args": {{"text": "My team is LASR from the United Kingdom at King's College London."}}}}]
JSON: {{"announcement": "I will tell you my team's affiliation."}}

Command: what day is today
Plan summary: answer question about today's date
Steps: [{{"skill": "say", "args": {{"text": "Today is Thursday, 3rd July 2026."}}}}]
JSON: {{"announcement": "I will tell you today's date."}}

Command: {command}
Plan summary: {plan_description}
Steps: {steps_json}
JSON: """


def parse_json(raw: str) -> dict:
    start = raw.find("{")
    end = raw.rfind("}") + 1
    if start == -1 or end == 0:
        return {}
    return json.loads(raw[start:end])
