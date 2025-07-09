#!/usr/bin/env python3
import itertools
import re

from typing import List, Union, TypedDict, Dict, Any

counter = 0
sub_command_counter = 0
seen_sub_command_group_names = []


def uniq(i: str) -> str:
    global counter
    counter += 1
    return f"uniq{counter}_{i}"


def list_to_regex(list: List[str], key: Union[str, None] = None):
    if key is None:
        return f"(?:{'|'.join(list)})"

    return f"(?P<{uniq(key)}>{'|'.join(list)})"


# data from gpsr_commands
verb_dict = {
    "take": ["take", "get", "grasp", "fetch"],
    "place": ["put", "place"],
    "deliver": ["bring", "give", "deliver"],
    "bring": ["bring", "give"],
    "go": ["go", "navigate"],
    "find": ["find", "locate", "look for"],
    "talk": ["tell", "say"],
    "answer": ["answer"],
    "meet": ["meet"],
    "tell": ["tell"],
    "greet": ["greet", "salute", "say hello to", "introduce yourself to"],
    # "remember": ["meet", "contact", "get to know", "get acquainted with"], <--- LOOKS UNUSED
    "count": ["tell me how many"],
    # "describe": ["tell me how", "describe"], <---- LOOKS UNUSED
    # "offer": ["offer"], <---- LOOKS UNUSED
    "follow": ["follow"],
    "guide": ["guide", "escort", "take", "lead"],
    # "accompany": ["accompany"], <---- LOOKS UNUSED
}


def verb(v):
    # return list_to_regex(verb_dict[v], f"verb_{v}")
    return list_to_regex(verb_dict[v], "verb")
    # return list_to_regex(verb_dict[v], None if len(verb_dict[v]) == 1 else "verb")


prep_dict = {
    "deliverPrep": ["to"],
    "placePrep": ["on"],
    "inLocPrep": ["in"],
    "fromLocPrep": ["from"],
    "toLocPrep": ["to"],
    "atLocPrep": ["at"],
    "talkPrep": ["to"],
    "locPrep": ["in", "at"],
    "onLocPrep": ["on"],
    "arePrep": ["are"],
    "ofPrsPrep": ["of"],
}


def prep(v: str):
    return list_to_regex(prep_dict[v])
    # return list_to_regex(prep_dict[v], f"prep_{v}")


class Configuration(TypedDict):
    person_names: List[str]
    location_names: List[str]
    placement_location_names: List[str]
    room_names: List[str]
    object_names: List[str]
    object_categories_plural: List[str]
    object_categories_singular: List[str]

    def key_to_list(self, key: str):
        if key == "name":
            return self["person_names"]
        elif key == "loc":
            return self["location_names"]
        elif key == "loc2":
            return self["location_names"]
            # return ['loc2']
        elif key == "plcmtLoc":
            return self["placement_location_names"]
        elif key == "plcmtLoc2":
            return self["placement_location_names"]
            # return ['plcmtLoc2']
        elif key == "room":
            return self["room_names"]
        elif key == "room2":
            return self["room_names"]
            # return ['room2']
        elif key == "obj":
            return self["object_names"]
        elif key == "singCat":
            return self["object_categories_singular"]
        elif key == "plurCat":
            return self["object_categories_plural"]
        else:
            raise Exception("unreachable")

    def pick(self, key: str, lists: List[str]):
        union = []
        for list in lists:
            if list == "inRoom":
                union.append(
                    f"(?:{prep('inLocPrep')} the (?P<{uniq('location')}>{'|'.join(Configuration.key_to_list(self, 'room'))}))"
                )
            elif list == "atLoc":
                union.append(
                    f"(?:{prep('atLocPrep')} the (?P<{uniq('location')}>{'|'.join(Configuration.key_to_list(self, 'loc'))}))"
                )
            else:
                union = union + Configuration.key_to_list(self, list)
        return f"(?P<{uniq(key)}>{'|'.join(union)})"


def gpsr_components():
    connector_list = ["and"]
    gesture_person_list = [
        "waving person",
        "person raising their left arm",
        "person raising their right arm",
        "person pointing to the left",
        "person pointing to the right",
    ]
    pose_person_list = ["sitting person", "standing person", "lying person"]
    # Ugly...
    gesture_person_plural_list = [
        "waving persons",
        "persons raising their left arm",
        "persons raising their right arm",
        "persons pointing to the left",
        "persons pointing to the right",
    ]
    pose_person_plural_list = ["sitting persons", "standing persons", "lying persons"]

    person_info_list = ["name", "pose", "gesture"]
    object_comp_list = [
        "biggest",
        "largest",
        "smallest",
        "heaviest",
        "lightest",
        "thinnest",
    ]

    talk_list = [
        "something about yourself",
        "the time",
        "what day is today",
        "what day is tomorrow",
        "your teams name",
        "your teams country",
        "your teams affiliation",
        "the day of the week",
        "the day of the month",
    ]
    question_list = ["question", "quiz"]

    color_list = [
        "blue",
        "yellow",
        "black",
        "white",
        "red",
        "orange",
        "gray",
    ]
    clothe_list = ["t shirt", "shirt", "blouse", "sweater", "coat", "jacket"]
    clothes_list = ["t shirts", "shirts", "blouses", "sweaters", "coats", "jackets"]
    color_clothe_list = []
    for a, b in list(itertools.product(color_list, clothe_list)):
        color_clothe_list = color_clothe_list + [a + " " + b]
    color_clothes_list = []
    for a, b in list(itertools.product(color_list, clothes_list)):
        color_clothes_list = color_clothes_list + [a + " " + b]

    # convert lists to regex components
    connector_list = list_to_regex(connector_list)
    gesture_person_list = list_to_regex(gesture_person_list, "gesture")
    pose_person_list = list_to_regex(pose_person_list, "pose")
    gesture_person_plural_list = list_to_regex(gesture_person_plural_list, "gesture")
    pose_person_plural_list = list_to_regex(pose_person_plural_list, "pose")
    person_info_list = list_to_regex(person_info_list, "personinfo")
    object_comp_list = list_to_regex(object_comp_list, "objectcomp")
    talk_list = list_to_regex(talk_list, "talk")
    question_list = list_to_regex(question_list, "question")
    color_clothe_list = list_to_regex(color_clothe_list, "clothes")
    color_clothes_list = list_to_regex(color_clothes_list, "clothes")

    return (
        verb,
        prep,
        "(?:an|a)",
        connector_list,
        gesture_person_list,
        pose_person_list,
        gesture_person_plural_list,
        pose_person_plural_list,
        person_info_list,
        object_comp_list,
        talk_list,
        question_list,
        color_clothe_list,
        color_clothes_list,
    )


def gpsr_regex(configuration: Configuration):
    (
        verb,
        prep,
        art,
        connector_list,
        gesture_person_list,
        pose_person_list,
        gesture_person_plural_list,
        pose_person_plural_list,
        person_info_list,
        object_comp_list,
        talk_list,
        question_list,
        color_clothe_list,
        color_clothes_list,
    ) = gpsr_components()

    commands = []

    def command(key: str, matcher: str):
        matcher = re.sub("\(\?P\<", f"(?P<CMD{key}_", matcher)
        commands.append(f"(?P<command_{key}>{matcher})")

    def get_possible_sub_commands(type: str) -> str:
        sub_commands = []
        assertion_check = False
        if type == "atLoc":
            sub_commands.append(
                f"{verb('find')} {art} {Configuration.pick(configuration, 'object', ['obj', 'singCat'])} and {get_possible_sub_commands('foundObj')}"
            )
            sub_commands.append(
                f"{verb('find')} the (?:{gesture_person_list}|{pose_person_list}) and {get_possible_sub_commands('foundPers')}"
            )
            sub_commands.append(
                f"{verb('meet')} {Configuration.pick(configuration, 'name', ['name'])} and {get_possible_sub_commands('foundPers')}"
            )
        elif type == "hasObj":
            sub_commands.append(
                f"{verb('place')} it {prep('onLocPrep')} the {Configuration.pick(configuration, 'location', ['plcmtLoc'])}"
            )
            sub_commands.append(f"{verb('deliver')} it to me")
            sub_commands.append(
                f"{verb('deliver')} it {prep('deliverPrep')} the (?:{gesture_person_list}|{pose_person_list}) {prep('inLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])}"
            )
            sub_commands.append(
                f"{verb('deliver')} it {prep('deliverPrep')} {Configuration.pick(configuration, 'name', ['name'])} {prep('inLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])}"
            )
        elif type == "foundPers":
            sub_commands.append(f"{verb('talk')} {talk_list}")
            sub_commands.append(f"{verb('answer')} a {question_list}")
            sub_commands.append(f"{verb('follow')} them")
            sub_commands.append(
                f"{verb('follow')} them {prep('toLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])}"
            )
            sub_commands.append(
                f"{verb('guide')} them {prep('toLocPrep')} the {Configuration.pick(configuration, 'location', ['room', 'loc'])}"
            )
        elif type == "foundObj":
            sub_commands.append(
                f"{verb('take')} it and {get_possible_sub_commands('hasObj')}"
            )
            assertion_check = True

        union = "|".join(sub_commands)
        group_names = re.findall(r"\(\?P<([a-zA-Z0-9_]+)>", union)
        global seen_sub_command_group_names
        global sub_command_counter
        for index, name in enumerate(group_names):
            if name in seen_sub_command_group_names:
                # make name unique
                new_name = f"{name}_{sub_command_counter}"
                seen_sub_command_group_names.append(new_name)
                sub_command_counter += 1
                union = re.sub(rf"\(\?P<{name}>", f"(?P<{new_name}>", union)
            else:
                seen_sub_command_group_names.append(name)

        # groups = re.search(r"(\b[A-Z]+\b).+(\b\d+)", union)
        return f"(?:{union})"
        # return f"(?P<{uniq(key)}>{'|'.join(union)})"

    # print(get_possible_sub_commands("atLoc"))
    command(
        "goToLoc",
        f"{verb('go')} {prep('toLocPrep')} the {Configuration.pick(configuration, 'location', ['loc', 'room'])} then {get_possible_sub_commands('atLoc')}",
    )
    command(
        "takeObjFromPlcmt",
        f"{verb('take')} {art} {Configuration.pick(configuration, 'object', ['obj', 'singCat'])} {prep('fromLocPrep')} the {Configuration.pick(configuration, 'location', ['plcmtLoc'])} and {get_possible_sub_commands('hasObj')}",
    )
    command(
        "findPrsInRoom",
        f"{verb('find')} a (?:{gesture_person_list}|{pose_person_list}) {prep('inLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])} and {get_possible_sub_commands('foundPers')}",
    )
    command(
        "findObjInRoom",
        f"{verb('find')} {art} {Configuration.pick(configuration, 'object', ['obj', 'singCat'])} {prep('inLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])} then {get_possible_sub_commands('foundObj')}",
    )
    command(
        "meetPrsAtBeac",
        f"{verb('meet')} {Configuration.pick(configuration, 'name', ['name'])} {prep('inLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])} and {get_possible_sub_commands('foundPers')}",
    )
    # command(
    #     "countObjOnPlcmt",
    #     f"{verb('count')} {Configuration.pick(configuration, 'object', ['plurCat'])} there are {prep('onLocPrep')} the {Configuration.pick(configuration, 'location', ['plcmtLoc'])}",
    # )

    # match either a category‐plural (e.g. "dishes") OR a specific object name (singular) with optional "s"
    plurals = "|".join(
        re.escape(cat) for cat in configuration["object_categories_plural"]
    )
    objs = "|".join(re.escape(obj) for obj in configuration["object_names"])

    command(
        "countObjOnPlcmt",
        rf"{verb('count')} (?P<object>(?:{plurals})|(?:{objs})s?) there are {prep('onLocPrep')} the {Configuration.pick(configuration, 'location', ['plcmtLoc'])}",
    )

    command(
        "countPrsInRoom",
        f"{verb('count')} (?:{gesture_person_plural_list}|{pose_person_plural_list}) are {prep('inLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])}",
    )
    command(
        "tellPrsInfoInLoc",
        f"{verb('tell')} me the {person_info_list} of the person {Configuration.pick(configuration, 'location', ['inRoom', 'atLoc'])}",
    )
    command(
        "tellObjPropOnPlcmt",
        f"{verb('tell')} me what is the {object_comp_list} object {prep('onLocPrep')} the {Configuration.pick(configuration, 'location', ['plcmtLoc'])}",
    )
    command(
        "talkInfoToGestPrsInRoom",
        f"{verb('talk')} {talk_list} {prep('talkPrep')} the {gesture_person_list} {prep('inLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])}",
    )
    command(
        "answerToGestPrsInRoom",
        f"{verb('answer')} the {question_list} {prep('ofPrsPrep')} the {gesture_person_list} {prep('inLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])}",
    )
    command(
        "followNameFromBeacToRoom",
        f"{verb('follow')} {Configuration.pick(configuration, 'name', ['name'])} {prep('fromLocPrep')} the {Configuration.pick(configuration, 'start', ['loc'])} {prep('toLocPrep')} the {Configuration.pick(configuration, 'end', ['room'])}",
    )
    command(
        "guideNameFromBeacToBeac",
        f"{verb('guide')} {Configuration.pick(configuration, 'name', ['name'])} {prep('fromLocPrep')} the {Configuration.pick(configuration, 'start', ['loc'])} {prep('toLocPrep')} the {Configuration.pick(configuration, 'end', ['loc', 'room'])}",
    )
    command(
        "guidePrsFromBeacToBeac",
        f"{verb('guide')} the (?:{gesture_person_list}|{pose_person_list}) {prep('fromLocPrep')} the {Configuration.pick(configuration, 'start', ['loc'])} {prep('toLocPrep')} the {Configuration.pick(configuration, 'end', ['loc', 'room'])}",
    )
    command(
        "guideClothPrsFromBeacToBeac",
        f"{verb('guide')} the person wearing a {color_clothe_list} {prep('fromLocPrep')} the {Configuration.pick(configuration, 'start', ['loc'])} {prep('toLocPrep')} the {Configuration.pick(configuration, 'end', ['loc', 'room'])}",
    )
    command(
        "bringMeObjFromPlcmt",
        f"{verb('bring')} me {art} {Configuration.pick(configuration, 'object', ['obj'])} {prep('fromLocPrep')} the {Configuration.pick(configuration, 'location', ['plcmtLoc'])}",
    )
    command(
        "tellCatPropOnPlcmt",
        f"{verb('tell')} me what is the {object_comp_list} {Configuration.pick(configuration, 'object', ['singCat'])} {prep('onLocPrep')} the {Configuration.pick(configuration, 'location', ['plcmtLoc'])}",
    )
    command(
        "greetClothDscInRm",
        f"{verb('greet')} the person wearing {art} {color_clothe_list} {prep('inLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])} and {get_possible_sub_commands('foundPers')}",
    )
    command(
        "greetNameInRm",
        f"{verb('greet')} {Configuration.pick(configuration, 'name', ['name'])} {prep('inLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])} and {get_possible_sub_commands('foundPers')}",
    )
    command(
        "meetNameAtLocThenFindInRm",
        f"{verb('meet')} {Configuration.pick(configuration, 'name', ['name'])} {prep('atLocPrep')} the {Configuration.pick(configuration, 'location', ['loc','room'])} then {verb('find')} them {prep('inLocPrep')} the {Configuration.pick(configuration, 'destination', ['room'])}",
    )
    command(
        "countClothPrsInRoom",
        f"{verb('count')} people {prep('inLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])} are wearing {color_clothes_list}",
    )
    command(
        "tellPrsInfoAtLocToPrsAtLoc",
        f"{verb('tell')} the {person_info_list} of the person {prep('atLocPrep')} the {Configuration.pick(configuration, 'location', ['loc','room'])} to the person {prep('atLocPrep')} the {Configuration.pick(configuration, 'destination', ['room', 'loc'])}",
    )
    command(
        "followPrsAtLoc",
        f"{verb('follow')} the (?:{gesture_person_list}|{pose_person_list}) {Configuration.pick(configuration, 'location', ['inRoom', 'atLoc'])}",
    )
    return "|".join(commands)


def gpsr_parse(matches: Dict[str, str]) -> Dict[str, Any]:
    result: dict[str, Any] = {
        "commands": [],
        "command_params": [],
    }
    for key in matches.keys():
        value = matches[key]
        if value is None:
            continue
        key_to_check = key.split("_")[-1]
        while key_to_check.isnumeric():
            key = "_".join(key.split("_")[:-1])
            key_to_check = key.split("_")[-1]
        if key_to_check == "verb":
            result["commands"].append(reverse_translate_verb_dict(value))
            result["command_params"].append({})

        elif key_to_check in [
            "object",
            "location",
            "gesture",
            "room",
            "name",
            "start",
            "end",
            "objectcomp",
            "personinfo",
            "clothes",
            "talk",
            "pose",
            "destination",
        ]:
            value_to_add = value
            try:
                result["command_params"][-1][key_to_check] = value_to_add
            except:
                continue
        else:
            pass
            # print(f"Unhandled key: {key_to_check}")
    return result


def gpsr_compile_and_parse(config: Configuration, input: Union[str, List[str]]) -> Dict:
    if isinstance(input, list):
        regex_str = gpsr_regex(config)
        regex = re.compile(regex_str)
        successful_parses = 0
        failed_parses = 0
        for inp in input:
            inp = inp.lower()

            # remove punctuation
            inp = re.sub(r"[^\w\s]", "", inp)
            if inp[0] == " ":
                inp = inp[1:]
            # print(input)
            # print(f"Parsed input: {input}")
            # print(f"possible configrations:{config}")
            # print(f"Regex: {regex}")
            matches = regex.match(inp)
            # print(matches)
            # print(f" matches:{matches}")
            try:
                matches = matches.groupdict()
            except:
                print(f"Failed to parse input: {inp}")
                failed_parses += 1
            object_categories = (
                config["object_categories_singular"]
                + config["object_categories_plural"]
            )
            successful_parses += 1
            # print(f"Successful parse {successful_parses} of {len(input)}: {inp}")
        print(
            f"Succeeded parse percentage: {successful_parses / len(input) * 100:.2f}%"
        )
        print(f"Failed parse percentage: {failed_parses / len(input) * 100:.2f}%")
    else:
        input = input.lower()

        # remove punctuation
        input = re.sub(r"[^\w\s]", "", input)
        if input[0] == " ":
            input = input[1:]
        # print(input)
        # print(f"Parsed input: {input}")
        regex_str = gpsr_regex(config)
        # print(f"possible configrations:{config}")
        regex = re.compile(regex_str)
        # print(f"Regex: {regex}")
        matches = regex.match(input)
        # print(matches)
        # print(f" matches:{matches}")
        matches = matches.groupdict()
        object_categories = (
            config["object_categories_singular"] + config["object_categories_plural"]
        )
        return parse_result_dict(
            gpsr_parse(matches), object_categories, config["room_names"]
        )
    return parse_result_dict(
        gpsr_parse(matches), object_categories, config["room_names"]
    )


def parse_result_dict(
    result: Dict, object_categories: List[str], rooms: List[str]
) -> Dict:
    """Parses the result dictionary output by the gpsr parse to
    handle missing parameters.

    Args:
        result (dict): _description_

    Returns:
        dict: _description_
    """
    for i, command in enumerate(result["commands"]):
        for key, value in result["command_params"][i].items():
            result["command_params"][i][key] = value.replace(" ", "_")

    for i, command in enumerate(result["commands"]):
        if "object" in result["command_params"][i]:
            if result["command_params"][i]["object"] in object_categories:
                # rename object to object category
                result["command_params"][i]["object_category"] = result[
                    "command_params"
                ][i]["object"]
                del result["command_params"][i]["object"]

        # Rename location to room if it is a room
        if "location" in result["command_params"][i]:
            if result["command_params"][i]["location"] in rooms:
                result["command_params"][i]["room"] = result["command_params"][i][
                    "location"
                ]
                del result["command_params"][i]["location"]

        # if "destination" in result["command_params"][i]:
        #     result["command_params"][i]["destination"] = result["command_params"][i][
        #             "destination"
        #         ].replace(" ", "_")
        #     print(result["command_params"][i]["destination"])

        # Update command params based on the previous commands params
        if i > 0:
            if "location" not in result["command_params"][i]:
                if "location" in result["command_params"][i - 1]:
                    result["command_params"][i]["location"] = result["command_params"][
                        i - 1
                    ]["location"]
            if "room" not in result["command_params"][i]:
                if "room" in result["command_params"][i - 1]:
                    result["command_params"][i]["room"] = result["command_params"][
                        i - 1
                    ]["room"]
            if "name" not in result["command_params"][i]:
                if "name" in result["command_params"][i - 1]:
                    result["command_params"][i]["name"] = result["command_params"][
                        i - 1
                    ]["name"]
            if "object" not in result["command_params"][i]:
                if "object" in result["command_params"][i - 1]:
                    result["command_params"][i]["object"] = result["command_params"][
                        i - 1
                    ]["object"]
            if "object_category" not in result["command_params"][i]:
                if "object_category" in result["command_params"][i - 1]:
                    result["command_params"][i]["object_category"] = result[
                        "command_params"
                    ][i - 1]["object_category"]

    return result


def reverse_translate_verb_dict(verb: str) -> str:
    for master_verb, verbs in verb_dict.items():
        if verb in verbs:
            return master_verb
    return verb


if __name__ == "__main__":
    object_categories_plural = [
        "dishes",
        "snacks",
        "fruits",
        "decorations",
        "drinks",
        "food",
        "cleaning supplies",
    ]
    object_categories_singular = [
        "dish",
        "snack",
        "fruit",
        "decoration",
        "drink",
        "food",
        "cleaning supply",
    ]
    object_categories = object_categories_singular + object_categories_plural
    config: Configuration = {
        "person_names": [
            "sophie",
            "julia",
            "emma",
            "sara",
            "laura",
            "hayley",
            "susan",
            "fleur",
            "gabrielle",
            "robin",
            "john",
            "liam",
            "lucas",
            "william",
            "kevin",
            "jesse",
            "noah",
            "harrie",
            "peter",
            "robin",
        ],
        "location_names": [
            "hallway cabinet",
            "entrance",
            "desk",
            "shelf",
            "coathanger",
            "exit",
            "TV table",
            "lounge chair",
            "lamp",
            "couch",
            "coffee table",
            "trashcan",
            "kitchen cabinet",
            "dinner table",
            "dishwasher",
            "kitchen counter",
        ],
        "placement_location_names": [
            "hallway cabinet",
            "desk",
            "shelf",
            "tv table",
            "coffee table",
            "kitchen cabinet",
            "dinner table",
            "dishwasher",
            "kitchen counter",
        ],
        "room_names": ["living room", "kitchen", "office", "hallway"],
        "object_names": [
            "soap",
            "dishwasher tab",
            "washcloth",
            "sponges",
            "cola",
            "ice tea",
            "water",
            "milk",
            "big coke",
            "fanta",
            "dubblefris",
            "cornflakes",
            "pea soup",
            "curry",
            "pancake mix",
            "hagelslag",
            "sausages",
            "mayonaise",
            "candle",
            "pear",
            "plum",
            "peach",
            "lemon",
            "orange",
            "strawberry",
            "banana",
            "apple",
            "stroopwafel",
            "candy",
            "liquorice",
            "crisps",
            "pringles",
            "tictac",
            "spoon",
            "plate",
            "cup",
            "fork",
            "bowl",
            "knife",
        ],
        "object_categories_plural": [
            "dishes",
            "snacks",
            "fruits",
            "decorations",
            "drinks",
            "food",
            "cleaning supplies",
        ],
        "object_categories_singular": [
            "dish",
            "snack",
            "fruit",
            "decoration",
            "drink",
            "food",
            "cleaning supply",
        ],
    }

    regex_str = gpsr_regex(config)

    regex = re.compile(regex_str)

    def execute(input: str, object_categories: List[str]):
        matches = regex.match(input).groupdict()
        return parse_result_dict(
            gpsr_parse(matches), object_categories, config["room_names"]
        )

    print(
        execute(
            "salute emma in the kitchen and tell the time",
            object_categories,
        )
    )

    # print(
    #     execute(
    #         "navigate to the kitchen then find a cup and get it and bring it to the person pointing to the right in the kitchen",
    #         object_categories,
    #     )
    # )
    #
    # print(
    #     execute(
    #         "tell the time to the person raising their right arm in the kitchen",
    #         object_categories,
    #     )
    # )
    # print(
    #     execute(
    #         "tell me the pose of the person at the sofa",
    #         object_categories,
    #     )
    # )
    #
    # print(
    #     execute(
    #         "navigate to the kitchen table then find a stick.",
    #         object_categories,
    #     )
    # )
    #
    # print(
    #     execute(
    #         "tell me what is the biggest stick on the kitchen table",
    #         object_categories,
    #     )
    # )
