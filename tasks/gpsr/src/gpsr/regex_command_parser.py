import itertools
import re

from typing import List, Union, TypedDict, Dict

counter = 0


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
    "remember": ["meet", "contact", "get to know", "get acquainted with"],
    "count": ["tell me how many"],
    "describe": ["tell me how", "describe"],
    "offer": ["offer"],
    "follow": ["follow"],
    "guide": ["guide", "escort", "take", "lead"],
    "accompany": ["accompany"],
}


def verb(v):
    # return list_to_regex(verb_dict[v], f"verb_{v}")
    return list_to_regex(verb_dict[v], None if len(verb_dict[v]) == 1 else "verb")


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

    color_list = ["blue", "yellow", "black", "white", "red", "orange", "gray"]
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

    def SUB_COMMAND_TODO_UNIMPLEMENTED():
        return f'(?P<{uniq("subcommand")}>.*)'

    command(
        "goToLoc",
        f"{verb('go')} {prep('toLocPrep')} the {Configuration.pick(configuration, 'location', ['loc', 'room'])} then {SUB_COMMAND_TODO_UNIMPLEMENTED()}",
    )
    command(
        "takeObjFromPlcmt",
        f"{verb('take')} {art} {Configuration.pick(configuration, 'object', ['obj', 'singCat'])} {prep('fromLocPrep')} the {Configuration.pick(configuration, 'location', ['plcmtLoc'])} and {SUB_COMMAND_TODO_UNIMPLEMENTED()}",
    )
    command(
        "findPrsInRoom",
        f"{verb('find')} a (?:{gesture_person_list}|{pose_person_list}) {prep('inLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])} and {SUB_COMMAND_TODO_UNIMPLEMENTED()}",
    )
    command(
        "findObjInRoom",
        f"{verb('find')} {art} {Configuration.pick(configuration, 'object', ['obj', 'singCat'])} {prep('inLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])} and {SUB_COMMAND_TODO_UNIMPLEMENTED()}",
    )
    command(
        "meetPrsAtBeac",
        f"{verb('meet')} {Configuration.pick(configuration, 'name', ['name'])} {prep('inLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])} and {SUB_COMMAND_TODO_UNIMPLEMENTED()}",
    )
    command(
        "countObjOnPlcmt",
        f"{verb('count')} {Configuration.pick(configuration, 'object', ['plurCat'])} there are {prep('onLocPrep')} the {Configuration.pick(configuration, 'location', ['plcmtLoc'])}",
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
        f"{verb('greet')} the person wearing {art} {color_clothe_list} {prep('inLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])} and {SUB_COMMAND_TODO_UNIMPLEMENTED()}",
    )
    command(
        "greetNameInRm",
        f"{verb('greet')} {Configuration.pick(configuration, 'name', ['name'])} {prep('inLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])} and {SUB_COMMAND_TODO_UNIMPLEMENTED()}",
    )
    command(
        "meetNameAtLocThenFindInRm",
        f"{verb('meet')} {Configuration.pick(configuration, 'name', ['name'])} {prep('atLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])} then {verb('find')} them {prep('inLocPrep')} the {Configuration.pick(configuration, 'destination', ['room'])}",
    )
    command(
        "countClothPrsInRoom",
        f"{verb('count')} people {prep('inLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])} are wearing {color_clothes_list}",
    )
    command(
        "tellPrsInfoAtLocToPrsAtLoc",
        f"{verb('tell')} the {person_info_list} of the person {prep('atLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])} to the person {prep('atLocPrep')} the {Configuration.pick(configuration, 'destination', ['room'])}",
    )
    command(
        "followPrsAtLoc",
        f"{verb('follow')} the (?:{gesture_person_list}|{pose_person_list}) {Configuration.pick(configuration, 'location', ['inRoom', 'atLoc'])}",
    )
    return "|".join(commands)


def gpsr_parse(matches: Dict[str, str]):
    result = {}
    for key in matches.keys():
        value = matches[key]
        if value is None:
            continue

        write_into = result
        key = re.sub("uniq\d+_", "", key)

        while key.startswith("CMD"):
            cmd, rest = key.split("_", 1)
            cmd = cmd[3:]  # remove CMD prefix

            if cmd not in write_into:
                write_into[cmd] = {}

            write_into = write_into[cmd]
            key = rest

        if "_" in key:
            actual_key, value = key.split("_")
            write_into[actual_key] = value
        else:
            write_into[key] = value
    return result


def gpsr_compile_and_parse(config: Configuration, input: str) -> dict:
    input = input.lower()
    # remove punctuation
    input = re.sub(r"[^\w\s]", "", input)
    print(input)
    if input[0] == " ":
        input = input[1:]

    regex_str = gpsr_regex(config)
    regex = re.compile(regex_str)
    matches = regex.match(input)
    matches = matches.groupdict()
    return gpsr_parse(matches)


if __name__ == "__main__":
    config: Configuration = {
        "person_names": ["guest1", "guest2"],
        "location_names": ["sofa", "piano"],
        "placement_location_names": ["kitchen table"],
        "room_names": ["living room", "kitchen"],
        "object_names": ["cup", "television"],
        "object_categories_plural": ["sticks"],
        "object_categories_singular": ["stick"],
    }

    regex_str = gpsr_regex(config)

    regex = re.compile(regex_str)

    def execute(input: str):
        matches = regex.match(input).groupdict()
        return gpsr_parse(matches)

    # subcommands aren't implemented but are caught:
    print(execute("go to the sofa then do something here"))
