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


def generate_command_start(self, cmd_category="", difficulty=0):
    cmd_list = []
    # cmd_list = ["goToLoc", "takeObjFromPlcmt", "findPrsInRoom", "findObjInRoom", "meetPrsAtBeac", "countObjOnPlcmt",
    #             "countPrsInRoom", "tellPrsInfoInLoc", "tellObjPropOnPlcmt", "talkInfoToGestPrsInRoom",
    #             "answerToGestPrsInRoom", "followNameFromBeacToRoom", "guideNameFromBeacToBeac",
    #             "guidePrsFromBeacToBeac", "guideClothPrsFromBeacToBeac", "bringMeObjFromPlcmt",
    #             "tellCatPropOnPlcmt", "greetClothDscInRm", "greetNameInRm", "meetNameAtLocThenFindInRm",
    #             "countClothPrsInRoom", "countClothPrsInRoom", "tellPrsInfoAtLocToPrsAtLoc", "followPrsAtLoc"]

    # HRI and people perception commands
    person_cmd_list = [
        "goToLoc",
        "findPrsInRoom",
        "meetPrsAtBeac",
        "countPrsInRoom",
        "tellPrsInfoInLoc",
        "talkInfoToGestPrsInRoom",
        "answerToGestPrsInRoom",
        "followNameFromBeacToRoom",
        "guideNameFromBeacToBeac",
        "guidePrsFromBeacToBeac",
        "guideClothPrsFromBeacToBeac",
        "greetClothDscInRm",
        "greetNameInRm",
        "meetNameAtLocThenFindInRm",
        "countClothPrsInRoom",
        "countClothPrsInRoom",
        "tellPrsInfoAtLocToPrsAtLoc",
        "followPrsAtLoc",
    ]
    # Object manipulation and perception commands
    object_cmd_list = [
        "goToLoc",
        "takeObjFromPlcmt",
        "findObjInRoom",
        "countObjOnPlcmt",
        "tellObjPropOnPlcmt",
        "bringMeObjFromPlcmt",
        "tellCatPropOnPlcmt",
    ]

    if cmd_category == "people":
        cmd_list = person_cmd_list
    elif cmd_category == "objects":
        cmd_list = object_cmd_list
    else:
        cmd_list = person_cmd_list if random.random() > 0.5 else object_cmd_list

    command = random.choice(cmd_list)
    # command = "" # To debug commands
    command_string = ""
    if command == "goToLoc":
        command_string = (
            "{goVerb} {toLocPrep} the {loc_room} then "
            + self.generate_command_followup("atLoc", cmd_category, difficulty)
        )
    elif command == "takeObjFromPlcmt":
        command_string = (
            "{takeVerb} {art} {obj_singCat} {fromLocPrep} the {plcmtLoc} and "
            + self.generate_command_followup("hasObj", cmd_category, difficulty)
        )
    elif command == "findPrsInRoom":
        command_string = (
            "{findVerb} a {gestPers_posePers} {inLocPrep} the {room} and "
            + self.generate_command_followup("foundPers", cmd_category, difficulty)
        )
    elif command == "findObjInRoom":
        command_string = (
            "{findVerb} {art} {obj_singCat} {inLocPrep} the {room} then "
            + self.generate_command_followup("foundObj", cmd_category, difficulty)
        )
    elif command == "meetPrsAtBeac":
        command_string = (
            "{meetVerb} {name} {inLocPrep} the {room} and "
            + self.generate_command_followup("foundPers", cmd_category, difficulty)
        )
    elif command == "countObjOnPlcmt":
        command_string = "{countVerb} {plurCat} there are {onLocPrep} the {plcmtLoc}"
    elif command == "countPrsInRoom":
        command_string = (
            "{countVerb} {gestPersPlur_posePersPlur} are {inLocPrep} the {room}"
        )
    elif command == "tellPrsInfoInLoc":
        command_string = "{tellVerb} me the {persInfo} of the person {inRoom_atLoc}"
    elif command == "tellObjPropOnPlcmt":
        command_string = (
            "{tellVerb} me what is the {objComp} object {onLocPrep} the {plcmtLoc}"
        )
    elif command == "talkInfoToGestPrsInRoom":
        command_string = (
            "{talkVerb} {talk} {talkPrep} the {gestPers} {inLocPrep} the {room}"
        )
    elif command == "answerToGestPrsInRoom":
        command_string = "{answerVerb} the {question} {ofPrsPrep} the {gestPers} {inLocPrep} the {room}"
    elif command == "followNameFromBeacToRoom":
        command_string = (
            "{followVerb} {name} {fromLocPrep} the {loc} {toLocPrep} the {room}"
        )
    elif command == "guideNameFromBeacToBeac":
        command_string = (
            "{guideVerb} {name} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}"
        )
    elif command == "guidePrsFromBeacToBeac":
        command_string = "{guideVerb} the {gestPers_posePers} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}"
    elif command == "guideClothPrsFromBeacToBeac":
        command_string = "{guideVerb} the person wearing a {colorClothe} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}"
    elif command == "bringMeObjFromPlcmt":
        command_string = "{bringVerb} me {art} {obj} {fromLocPrep} the {plcmtLoc}"
    elif command == "tellCatPropOnPlcmt":
        command_string = (
            "{tellVerb} me what is the {objComp} {singCat} {onLocPrep} the {plcmtLoc}"
        )
    elif command == "greetClothDscInRm":
        command_string = (
            "{greetVerb} the person wearing {art} {colorClothe} {inLocPrep} the {room} and "
            + self.generate_command_followup("foundPers", cmd_category, difficulty)
        )
    elif command == "greetNameInRm":
        command_string = (
            "{greetVerb} {name} {inLocPrep} the {room} and "
            + self.generate_command_followup("foundPers", cmd_category, difficulty)
        )
    elif command == "meetNameAtLocThenFindInRm":
        command_string = "{meetVerb} {name} {atLocPrep} the {loc} then {findVerb} them {inLocPrep} the {room}"
    elif command == "countClothPrsInRoom":
        command_string = (
            "{countVerb} people {inLocPrep} the {room} are wearing {colorClothes}"
        )
    elif command == "countClothPrsInRoom":
        command_string = (
            "{countVerb} people {inLocPrep} the {room} are wearing {colorClothes}"
        )
    elif command == "tellPrsInfoAtLocToPrsAtLoc":
        command_string = "{tellVerb} the {persInfo} of the person {atLocPrep} the {loc} to the person {atLocPrep} the {loc2}"
    elif command == "followPrsAtLoc":
        command_string = "{followVerb} the {gestPers_posePers} {inRoom_atLoc}"
    else:
        warnings.warn("Command type not covered: " + command)
        return "WARNING"

    for ph in re.findall(r"(\{\w+\})", command_string, re.DOTALL):
        command_string = command_string.replace(ph, self.insert_placeholders(ph))

    # TODO allow multiple articles
    art_ph = re.findall(r"\{(art)\}\s*([A-Za-z])", command_string, re.DOTALL)
    if art_ph:
        command_string = command_string.replace(
            "art", "an" if art_ph[0][1].lower() in ["a", "e", "i", "o", "u"] else "a"
        )
    # TODO eliminate double mentions of location
    if "loc2" in command_string:
        command_string = command_string.replace(
            "loc2",
            random.choice([x for x in self.location_names if x not in command_string]),
        )
    elif "room2" in command_string:
        command_string = command_string.replace(
            "room2",
            random.choice([x for x in self.room_names if x not in command_string]),
        )
    elif "plcmtLoc2" in command_string:
        command_string = command_string.replace(
            "plcmtLoc2",
            random.choice(
                [x for x in self.placement_location_names if x not in command_string]
            ),
        )
    return command_string.replace("{", "").replace("}", "")


def generate_command_followup(self, type, cmd_category="", difficulty=0):
    if type == "atLoc":
        person_cmd_list = ["findPrs", "meetName"]
        object_cmd_list = ["findObj"]
        if cmd_category == "people":
            cmd_list = person_cmd_list
        elif cmd_category == "objects":
            cmd_list = object_cmd_list
        else:
            cmd_list = person_cmd_list if random.random() > 0.5 else object_cmd_list
    elif type == "hasObj":
        cmd_list = [
            "placeObjOnPlcmt",
            "deliverObjToMe",
            "deliverObjToPrsInRoom",
            "deliverObjToNameAtBeac",
        ]
    elif type == "foundPers":
        cmd_list = [
            "talkInfo",
            "answerQuestion",
            "followPrs",
            "followPrsToRoom",
            "guidePrsToBeacon",
        ]
    elif type == "foundObj":
        cmd_list = ["takeObj"]

    command = random.choice(cmd_list)
    command_string = ""
    if command == "findObj":
        command_string = (
            "{findVerb} {art} {obj_singCat} and "
            + self.generate_command_followup("foundObj")
        )
    elif command == "findPrs":
        command_string = (
            "{findVerb} the {gestPers_posePers} and "
            + self.generate_command_followup("foundPers")
        )
    elif command == "meetName":
        command_string = "{meetVerb} {name} and " + self.generate_command_followup(
            "foundPers"
        )
    elif command == "placeObjOnPlcmt":
        command_string = "{placeVerb} it {onLocPrep} the {plcmtLoc2}"
    elif command == "deliverObjToMe":
        command_string = "{deliverVerb} it to me"
    elif command == "deliverObjToPrsInRoom":
        command_string = "{deliverVerb} it {deliverPrep} the {gestPers_posePers} {inLocPrep} the {room}"
    elif command == "deliverObjToNameAtBeac":
        command_string = "{deliverVerb} it {deliverPrep} {name} {inLocPrep} the {room}"
    elif command == "talkInfo":
        command_string = "{talkVerb} {talk}}"
    elif command == "answerQuestion":
        command_string = "{answerVerb} a {question}"
    elif command == "followPrs":
        command_string = "{followVerb} them"
    elif command == "followPrsToRoom":
        command_string = "{followVerb} them {toLocPrep} the {loc2_room2}"
    elif command == "guidePrsToBeacon":
        command_string = "{guideVerb} them {toLocPrep} the {loc2_room2}"
    elif command == "takeObj":
        command_string = "{takeVerb} it and " + self.generate_command_followup("hasObj")
    else:
        warnings.warn("Command type not covered: " + command)
        return "WARNING"
    return command_string


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

    def get_possible_sub_commands(type: str) -> str:
        sub_commands = []
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
                f"{verb('guide')} them {prep('toLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])}"
            )
        elif type == "foundObj":
            sub_commands.append(
                f"{verb('take')} it and {get_possible_sub_commands('hasObj')}"
            )

        union = "|".join(sub_commands)
        return union

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
        f"{verb('find')} {art} {Configuration.pick(configuration, 'object', ['obj', 'singCat'])} {prep('inLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])} and {get_possible_sub_commands('foundObj')}",
    )
    command(
        "meetPrsAtBeac",
        f"{verb('meet')} {Configuration.pick(configuration, 'name', ['name'])} {prep('inLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])} and {get_possible_sub_commands('foundPers')}",
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
        f"{verb('greet')} the person wearing {art} {color_clothe_list} {prep('inLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])} and {get_possible_sub_commands('foundPers')}",
    )
    command(
        "greetNameInRm",
        f"{verb('greet')} {Configuration.pick(configuration, 'name', ['name'])} {prep('inLocPrep')} the {Configuration.pick(configuration, 'location', ['room'])} and {get_possible_sub_commands('foundPers')}",
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
    print(
        execute(
            "locate a cup in the kitchen then fetch it and bring it to the person raising their right arm in the kitchen"
        )
    )
