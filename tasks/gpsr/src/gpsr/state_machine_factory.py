#!/usr/bin/env python3
import os
import rospy
import smach
import smach_ros
import rospkg
from typing import Dict, List

from lasr_skills import (
    GoToLocation,
    FindPerson,
    Guide,
    HandoverObject,
    Say,
    ReceiveObject,
    FindPersonAndTell,
    CountPeople,
    LookToPoint,
)

# from lasr_person_following.msg import FollowAction, FollowGoal

from gpsr.states import (
    Talk,
    QuestionAnswer,
    GoFindTheObject,
    ObjectComparison,
    CountObject,
)

from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
    Polygon,
    PoseWithCovarianceStamped,
    PointStamped,
)
from std_msgs.msg import Header

STATE_COUNT = 0

OBJECT_CATEGORIES = {
    "cleaning_supplies": ["soap", "dishwasher tab", "washcloth", "sponges"],
    "cleaning_supply": ["soap", "dishwasher tab", "washcloth", "sponges"],
    "drinks": ["cola", "ice_tea", "water", "milk", "big_coke", "fanta", "dubbelfris"],
    "drink": ["cola", "ice_tea", "water", "milk", "big_coke", "fanta", "dubbelfris"],
    "food": [
        "cornflakes",
        "pea_soup",
        "curry",
        "pancake_mix",
        "hagelslag",
        "sausages",
        "mayonaise",
    ],
    "decorations": ["candle"],
    "decoration": ["candle"],
    "fruits": [
        "pear",
        "plum",
        "peach",
        "lemon",
        "orange",
        "strawberry",
        "banana",
        "apple",
    ],
    "fruit": [
        "pear",
        "plum",
        "peach",
        "lemon",
        "orange",
        "strawberry",
        "banana",
        "apple",
    ],
    "snacks": ["stroopwafel", "candy", "liquorice", "crisps", "pringles", "tictac"],
    "snack": ["stroopwafel", "candy", "liquorice", "crisps", "pringles", "tictac"],
    "dishes": ["spoon", "plate", "cup", "fork", "bowl", "knife"],
    "dish": ["spoon", "plate", "cup", "fork", "bowl", "knife"],
}


OBJECT_CATEGORY_LOCATIONS = {
    "decorations": "desk",
    "decoration": "desk",
    "cleaning_supplies": "shelf",
    "cleaning_supply": "shelf",
    "toys": "tv_table",
    "toy": "tv_table",
    "fruits": "coffee_table",
    "fruit": "coffee_table",
    "drinks": "kitchen_cabinet",
    "drink": "kitchen_cabinet",
    "snacks": "dinner_table",
    "snack": "dinner_table",
    "dishes": "dishwasher",
    "dish": "dishwasher",
}


"""
Helpers
"""


# right after your imports:
class ClothingString(str):
    """
    A str subclass whose .split property returns [colour, garment],
    with:
      - underscores/hyphens → spaces
      - lowercasing
      - trailing 's' stripped from the garment
    """

    def __new__(cls, raw: str):
        norm = raw.replace("_", " ").replace("-", " ").lower()
        return super().__new__(cls, norm)

    @property
    def split(self):
        # split into [colour, rest_of_garment]
        parts = super().split(None, 1)
        if len(parts) == 1:
            parts.append("")
        # singularize plural garments: "t shirts" → "t shirt"
        col, cloth = parts
        if cloth.endswith("s"):
            cloth = cloth[:-1]
        return [col, cloth]


def increment_state_count() -> int:
    global STATE_COUNT
    STATE_COUNT += 1
    return STATE_COUNT


def get_location_room(location: str) -> str:
    rooms = rospy.get_param("/gpsr/arena/rooms")
    for room in rooms:
        if location in rooms[room]["beacons"]:
            return room
    raise ValueError(f"Location {location} not found in the arena")


def get_current_pose() -> Pose:
    pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
    return pose.pose.pose


def get_location_pose(location: str, person: bool, dem_manipulation=False) -> Pose:
    location_room = get_location_room(location)
    if person:
        location_pose: Dict = rospy.get_param(
            f"/gpsr/arena/rooms/{location_room}/beacons/{location}/person_detection_pose"
        )
    elif not dem_manipulation:
        location_pose: Dict = rospy.get_param(
            f"/gpsr/arena/rooms/{location_room}/beacons/{location}/object_detection_pose"
        )
    else:
        location_pose: Dict = rospy.get_param(
            f"/gpsr/arena/rooms/{location_room}/beacons/{location}/dem_manipulation_pose"
        )

    return Pose(
        position=Point(**location_pose["position"]),
        orientation=Quaternion(**location_pose["orientation"]),
    )


def get_room_pose(room: str) -> Pose:
    location_pose: Dict = rospy.get_param(f"/gpsr/arena/rooms/{room}/pose")
    return Pose(
        position=Point(**location_pose["position"]),
        orientation=Quaternion(**location_pose["orientation"]),
    )


def get_object_detection_poses(room: str) -> List[Pose]:
    poses = []

    for beacon in rospy.get_param(f"/gpsr/arena/rooms/{room}/beacons"):
        if "object_detection_pose" in beacon:
            poses.append(
                Pose(
                    position=Point(**beacon["object_detection_pose"]["position"]),
                    orientation=Quaternion(
                        **beacon["object_detection_pose"]["orientation"]
                    ),
                )
            )
    return poses


def get_look_detection_poses(room: str) -> List[Point]:
    poses = []

    for beacon in rospy.get_param(f"/gpsr/arena/rooms/{room}/beacons"):
        if "object_detection_point" in beacon:
            poses.append(
                Point(**beacon["object_detection_point"]),
            )
    return poses


def get_person_detection_poses(room: str) -> List[Pose]:
    poses = []
    for pose in rospy.get_param(f"/gpsr/arena/rooms/{room}/people_search_poses"):
        poses.append(
            Pose(
                position=Point(**pose["position"]),
                orientation=Quaternion(**pose["orientation"]),
            )
        )
    return poses


def get_room_polygon(room: str) -> Polygon:
    return Polygon(
        [
            Point(p[0], p[1], 0.0)
            for p in rospy.get_param(f"/gpsr/arena/rooms/{room}/room_polygon")
        ]
    )


def get_look_point(location: str) -> PointStamped:
    location_room = get_location_room(location)
    look_point = rospy.get_param(
        f"/gpsr/arena/rooms/{location_room}/beacons/{location}/object_detection_point"
    )
    return PointStamped(
        header=Header(frame_id="map"),
        point=Point(**look_point),
    )


def get_objects_from_category(object_name: str) -> str:
    object_name = object_name.lower().strip()
    if object_name.endswith("s"):
        singular = object_name[:-1]
    else:
        singular = object_name

    object_groups = rospy.get_param("/gpsr/objects")
    for group in object_groups:
        for category, items in group.items():
            if object_name in items or singular in items:
                return category
    raise ValueError(f"Object '{object_name}' not found in any category.")


"""
Verbs
"""


def greet(command_param: Dict, sm: smach.StateMachine) -> None:
    location_string = ""
    if "room" in command_param:
        waypoints: List[Pose] = get_person_detection_poses(command_param["room"])
        polygon: Polygon = get_room_polygon(command_param["room"])
        location_string = command_param["room"]
    elif "destination" in command_param:
        waypoints: List[Pose] = get_person_detection_poses(command_param["destination"])
        polygon: Polygon = get_room_polygon(command_param["destination"])
        location_string = command_param["destination"]
    elif "location" in command_param:
        waypoints: List[Pose] = [get_location_pose(command_param["location"], True)]
        polygon: Polygon = get_room_polygon(
            get_location_room(command_param["location"])
        )
        location_string = command_param["location"]
    else:
        raise ValueError(
            "Greet command received with no room or location in command parameters"
        )
    output_string = "I will greet "
    if "name" in command_param:
        criteria = "name"
        criteria_value = command_param["name"]
        output_string += f"{criteria_value} "

    elif "clothes" in command_param:
        criteria = "clothes"
        # normalize exactly as in count()
        norm = command_param["clothes"].replace("_", " ").replace("-", " ").lower()
        parts = norm.split()
        if parts[-1].endswith("s"):
            parts[-1] = parts[-1][:-1]
        criteria_value = " ".join(parts)
        find_person = True
        output_string += f"the person wearing a {criteria_value} "

    elif "gesture" in command_param:
        criteria = "gesture"
        criteria_value = command_param["gesture"]
        output_string += f"the {criteria_value} "
        if "pointing" in criteria_value:
            if "left" in criteria_value:
                criteria_value = "pointing_to_the_left"
            elif "right" in criteria_value:
                criteria_value = "pointing_to_the_right"
        elif "raising" in criteria_value:
            if "left" in criteria_value:
                criteria_value = "raising_left_arm"
            elif "right" in criteria_value:
                criteria_value = "raising_right_arm"
        elif "waving" in criteria_value:
            criteria_value = "waving"
        rospy.loginfo(f"CRITERIA VALUE: {criteria_value}")
    elif "pose" in command_param:
        criteria = "pose"
        criteria_value = command_param["pose"]
        output_string += f"the {criteria_value} "
    else:
        raise ValueError(
            "Greet command received with no name, clothes, gesture, or pose in command parameters"
        )

    output_string += f"at the {location_string}"
    sm.add(
        f"STATE_{increment_state_count()}",
        Say(text=output_string),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "preempted": f"STATE_{STATE_COUNT + 1}",
            "aborted": f"STATE_{STATE_COUNT + 1}",
        },
    )
    sm.add(
        f"STATE_{increment_state_count()}",
        FindPerson(
            waypoints=waypoints,
            polygon=polygon,  # Polygon of the room to find the person in.
            criteria=criteria,
            criteria_value=criteria_value,
        ),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": f"STATE_{STATE_COUNT + 1}",
        },
    )

    # Then say the greeting
    sm.add(
        f"STATE_{increment_state_count()}",
        Say(text=f"Hello to the person {criteria_value} in the {location_string}"),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "preempted": f"STATE_{STATE_COUNT + 1}",
            "aborted": f"STATE_{STATE_COUNT + 1}",
        },
    )


def talk(command_param: Dict, sm: smach.StateMachine, greet_person: bool) -> None:
    """
    This combines talk and tell as they use the same verb.

    Talk:
        - Say a given response
        - Say a given response to a person in a given gesture at a given location.

    Tell:
        People

        Need additional find person skill, then handle the following commands here

        - Tell me the name of the person at a given room or location.
        - Tell me the pose of the person at a given room or location.
        - Tell me the gesture of the person at a given room or location.

        Objects

        This can all be a single skill that takes img_msg, runs inference, outputs object name

        - Tell me what is the biggest object at a given placement location
        - Tell me what is the largest object at a given placement location
        - Tell me what is the smallest object at a given placement location
        - Tell me what is the heaviest object at a given placement location
        - Tell me what is the lightest object at a given placement location
        - Tell me what is the thinnest object at a given placement location

        - Tell me what is the biggest object in a given category at a given placement location
        - Tell me what is the largest object in a given category at a given placement location
        - Tell me what is the smallest object in a given category at a given placement location
        - Tell me what is the heaviest object in a given category at a given placement location
        - Tell me what is the lightest object in a given category at a given placement location
        - Tell me what is the thinnest object at in a given category a given placement location


    """

    if "talk" in command_param:
        if greet_person:
            greet(command_param, sm)
        sm.add(
            f"STATE_{increment_state_count()}",
            Talk(command_param["talk"]),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "failed": f"STATE_{STATE_COUNT + 1}",
            },
        )
    elif "personinfo" in command_param:
        query: str = command_param["personinfo"]
        if query not in ["name", "pose", "gesture"]:
            raise ValueError(
                f"Person info query {query} not recognised. Must be 'name', 'pose', or 'gesture'"
            )

        output_str = "I will determine the " + query + " of the person"

        if "room" in command_param:
            waypoints: List[Pose] = get_person_detection_poses(command_param["room"])
            polygon: Polygon = get_room_polygon(command_param["room"])
            location_str = command_param["room"]
        elif "location" in command_param:
            waypoints: List[Pose] = [get_location_pose(command_param["location"], True)]
            polygon: Polygon = get_room_polygon(
                get_location_room((command_param["location"]))
            )
            location_str = command_param["location"]
        else:
            raise ValueError(
                "Tell command received with personinfo, but no room or location in command parameters"
            )
        output_str += f" at the {location_str}"
        sm.add(
            f"STATE_{increment_state_count()}",
            Say(text=output_str),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "preempted": f"STATE_{STATE_COUNT + 1}",
                "aborted": f"STATE_{STATE_COUNT + 1}",
            },
        )
        sm.add(
            f"STATE_{increment_state_count()}",
            FindPersonAndTell(
                waypoints=waypoints,
                polygon=polygon,
                criteria=query,
            ),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "failed": f"STATE_{STATE_COUNT + 1}",
            },
            remapping={"query_result": "query_result"},
        )

        if "destination" in command_param:
            location_pose = get_location_pose(command_param["destination"], True)
        else:
            location_pose = get_current_pose()

        # maybe: find person?
        sm.add(
            f"STATE_{increment_state_count()}",
            GoToLocation(location=location_pose),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "failed": f"STATE_{STATE_COUNT + 1}",
            },
        )

        sm.add(
            f"STATE_{increment_state_count()}",
            Say(format_str="The " + query + " of the person is {}"),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "preempted": f"STATE_{STATE_COUNT + 1}",
                "aborted": f"STATE_{STATE_COUNT + 1}",
            },
            remapping={"placeholders": "query_result"},
        )

    elif "objectcomp" in command_param:

        query = command_param["objectcomp"]
        output_str = f"I will determine the {query} of the specified object"
        if "location" in command_param:
            location_pose = get_location_pose(command_param["location"], False)
            look_point = get_look_point(command_param["location"])
            area_polygon = get_room_polygon(
                get_location_room((command_param["location"]))
            )
        else:
            raise ValueError(
                "Tell command with object but no room in command parameters"
            )

        output_str += f" in the {command_param['location']}"
        sm.add(
            f"STATE_{increment_state_count()}",
            Say(text=output_str),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "preempted": f"STATE_{STATE_COUNT + 1}",
                "aborted": f"STATE_{STATE_COUNT + 1}",
            },
        )
        sm.add(
            f"STATE_{increment_state_count()}",
            GoToLocation(location=location_pose),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "failed": f"STATE_{STATE_COUNT + 1}",
            },
        )

        sm.add(
            f"STATE_{increment_state_count()}",
            LookToPoint(pointstamped=look_point),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "aborted": f"STATE_{STATE_COUNT + 1}",
                "timed_out": f"STATE_{STATE_COUNT + 1}",
            },
        )

        if "object_category" in command_param:
            objects = get_objects_from_category(command_param["object_category"])
        else:
            objects = None

        sm.add(
            f"STATE_{increment_state_count()}",
            ObjectComparison(query=query, area_polygon=area_polygon, objects=objects),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "failed": f"STATE_{STATE_COUNT + 1}",
            },
        )

        sm.add(
            f"STATE_{increment_state_count()}",
            GoToLocation(location=get_current_pose()),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "failed": f"STATE_{STATE_COUNT + 1}",
            },
        )

        sm.add(
            f"STATE_{increment_state_count()}",
            Say(format_str="The " + query + " is {}"),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "preempted": f"STATE_{STATE_COUNT + 1}",
                "aborted": f"STATE_{STATE_COUNT + 1}",
            },
            remapping={"placeholders": "query_result"},
        )

    else:
        raise ValueError(
            "Tell command received with no talk, personinfo or objectcomp in command parameters"
        )


def guide(command_param: Dict, sm: smach.StateMachine) -> None:
    """
    Guide commands can be of the following form:

    - Guide a person in front of the robot to a given room.
    - Guide a named person from a start location to a destination room or location.
    - Guide a person in a given gesture from a given start location to a destination room or location.
    - Guide a person in a given pose from a given start location to a destination room or location.
    - Guide a person in a given set of clothes from a given start location to a destination room or location.

    """

    find_person = False
    output_string = "I will find "
    if "start" not in command_param:
        pass
    elif "name" in command_param:
        criteria = "name"
        criteria_value = command_param["name"]
        output_string += f"{criteria_value} "
        find_person = True
    elif "clothes" in command_param:
        criteria = "clothes"
        # normalize exactly as in count()
        norm = command_param["clothes"].replace("_", " ").replace("-", " ").lower()
        parts = norm.split()
        if parts[-1].endswith("s"):
            parts[-1] = parts[-1][:-1]
        criteria_value = " ".join(parts)
        find_person = True
        output_string += f"the person wearing a {criteria_value} "

    elif "gesture" in command_param:
        criteria = "gesture"
        criteria_value = command_param["gesture"]
        find_person = True
        output_string += f"the {criteria_value} "
    elif "pose" in command_param:
        criteria = "pose"
        criteria_value = command_param["pose"]
        find_person = True
        output_string += f"the {criteria_value} "
    else:
        location_pose = get_room_pose(command_param["room"])
        location_name = command_param["room"]

    if find_person:
        start_loc = command_param["start"]
        output_string += f"at the {start_loc}"
        start_pose = get_location_pose(start_loc, person=True)
        room = get_location_room(start_loc)
        polygon: Polygon = get_room_polygon(room)

        sm.add(
            f"STATE_{increment_state_count()}",
            Say(text=output_string),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "preempted": f"STATE_{STATE_COUNT + 1}",
                "aborted": f"STATE_{STATE_COUNT + 1}",
            },
        )
        sm.add(
            f"STATE_{increment_state_count()}",
            FindPerson(
                waypoints=[start_pose],
                polygon=polygon,
                criteria=criteria,
                criteria_value=criteria_value,
            ),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "failed": f"STATE_{STATE_COUNT + 1}",
                # "failed": f"NO_PERSON",
            },
        )

        # NO_PERSON: if the sub‐SM failed, greet that no one was found and finish
        sm.add(
            "NO_PERSON",
            Say(
                text=f"Sorry, I couldn’t find anyone wearing {criteria_value} at the {start_loc}."
            ),
            transitions={
                "succeeded": "succeeded",
                "aborted": "succeeded",
                "preempted": "succeeded",
            },
        )

        try:
            location_pose = get_location_pose(command_param["end"], person=True)
            location_name = command_param["end"]
        except KeyError:
            location_pose = get_room_pose(command_param["end"])
            location_name = command_param["end"]
    else:
        if "room" in command_param:
            location_pose = get_room_pose(command_param["room"])
            location_name = command_param["room"]
        elif "location" in command_param:
            location_pose = get_location_pose(command_param["location"], True)
            location_name = command_param["location"]
        else:
            raise ValueError(
                "Guide command received with no room or location in command parameters"
            )

    sm.add(
        f"STATE_{increment_state_count()}",
        Guide(location_name=location_name, location_pose=location_pose),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": f"STATE_{STATE_COUNT + 1}",
        },
    )


def deliver(command_param: Dict, sm: smach.StateMachine) -> None:
    """
    The possible deliver commands are:
        - deliver an object the robot aleady has to me
        - deliver an object the robot already has to a person in a given gesture in a given room.
        - deliver an object the robot already has to a person in a given pose in a given room.
        - deliver an object the robot already has to a person with a given name in a given room.


    """

    if "room" in command_param:
        waypoints: List[Pose] = get_person_detection_poses(command_param["room"])
        polygon: Polygon = get_room_polygon(command_param["room"])
    else:
        raise ValueError("Deliver command received with no room in command parameters")

    if "name" in command_param:
        criteria = "name"
        criteria_value = command_param["name"]
        output_string = f"I will deliver the object to {criteria_value}"
    elif "gesture" in command_param:
        criteria = "gesture"
        criteria_value = command_param["gesture"]
        output_string = f"I will deliver the object to the {criteria_value}"
    elif "pose" in command_param:
        criteria = "pose"
        criteria_value = command_param["pose"]
        output_string = f"I will deliver the object to the {criteria_value}"
    else:
        criteria = None
        waypoints = [get_current_pose()]
    output_string += f" in the {command_param['room']}"

    if "object" in command_param:
        object_name = command_param["object"]
    elif "object_category" in command_param:
        object_name = command_param["object_category"]
    else:
        raise ValueError(
            "Deliver command received with no object or object category in command parameters"
        )
    output_string.replace("object", object_name)

    sm.add(
        f"STATE_{increment_state_count()}",
        Say(text=output_string),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "preempted": f"STATE_{STATE_COUNT + 1}",
            "aborted": f"STATE_{STATE_COUNT + 1}",
        },
    )
    sm.add(
        f"STATE_{increment_state_count()}",
        FindPerson(
            waypoints=waypoints,
            polygon=polygon,
            criteria=criteria,
            criteria_value=criteria_value,
        ),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": f"STATE_{STATE_COUNT + 1}",
        },
    )
    sm.add(
        f"STATE_{increment_state_count()}",
        HandoverObject(object_name=object_name),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": f"STATE_{STATE_COUNT + 1}",
        },
    )


def place(command_param: Dict, sm: smach.StateMachine) -> None:

    if "location" in command_param:
        location_pose = get_location_pose(
            command_param["location"], False, dem_manipulation=True
        )
    else:
        raise ValueError(
            "Place command received with no location in command parameters"
        )

    if "object" in command_param:
        object_name = command_param["object"]
    elif "object_category" in command_param:
        object_name = command_param["object_category"]
    else:
        raise ValueError(
            "Place command received with no object or object category in command parameters"
        )

    sm.add(
        f"STATE_{increment_state_count()}",
        Say(
            text=f"I am going to place the {object_name} at the {command_param['location']}"
        ),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "aborted": f"STATE_{STATE_COUNT + 1}",
            "preempted": f"STATE_{STATE_COUNT + 1}",
        },
    )
    sm.add(
        f"STATE_{increment_state_count()}",
        GoToLocation(location=location_pose),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": f"STATE_{STATE_COUNT + 1}",
        },
    )
    sm.add(
        f"STATE_{increment_state_count()}",
        HandoverObject(object_name=object_name),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": f"STATE_{STATE_COUNT + 1}",
        },
    )

    sm.add(
        f"STATE_{increment_state_count()}",
        Say(text=f"Please place the object on the {command_param['location']}"),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "aborted": f"STATE_{STATE_COUNT + 1}",
            "preempted": f"STATE_{STATE_COUNT + 1}",
        },
    )


def take(command_param: Dict, sm: smach.StateMachine) -> None:

    if "object" in command_param:
        criteria = "object"
        criteria_value = command_param["object"]
    elif "object_category" in command_param:
        criteria = "object_category"
        criteria_value = command_param["object_category"]
    else:
        raise ValueError(
            "Take command received with no object or object category in command parameters"
        )
    if "location" in command_param:
        location_pose = get_location_pose(command_param["location"], False)
    else:
        raise ValueError("Take command received with no location in command parameters")

    # TODO: this should use find, to find the object at the location
    sm.add(
        f"STATE_{increment_state_count()}",
        Say(
            text=f"I will take the {criteria_value} from the {command_param['location']}"
        ),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "aborted": f"STATE_{STATE_COUNT + 1}",
            "preempted": f"STATE_{STATE_COUNT + 1}",
        },
    )
    sm.add(
        f"STATE_{increment_state_count()}",
        GoToLocation(location=location_pose),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": f"STATE_{STATE_COUNT + 1}",
        },
    )

    sm.add(
        f"STATE_{increment_state_count()}",
        Say(
            text=f"Please pick up the {criteria_value} on the {command_param['location']} for me."
        ),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "aborted": f"STATE_{STATE_COUNT + 1}",
            "preempted": f"STATE_{STATE_COUNT + 1}",
        },
    )

    sm.add(
        f"STATE_{increment_state_count()}",
        GoToLocation(
            location=get_location_pose(
                command_param["location"], False, dem_manipulation=True
            ),
        ),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": f"STATE_{STATE_COUNT + 1}",
        },
    )

    sm.add(
        f"STATE_{increment_state_count()}",
        ReceiveObject(object_name=criteria_value),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": f"STATE_{STATE_COUNT + 1}",
        },
    )


def answer(command_param: Dict, sm: smach.StateMachine, greet_person: bool) -> None:
    data_root = os.path.join(rospkg.RosPack().get_path("gpsr"), "data", "qa_data")

    if greet_person:
        greet(command_param, sm)

    sm.add(
        f"STATE_{increment_state_count()}",
        QuestionAnswer(
            index_path=os.path.join(data_root, "questions.index"),
            txt_path=os.path.join(data_root, "questions.txt"),
            json_path=os.path.join(data_root, "qa_mapping.json"),
            k=1,
        ),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": f"STATE_{STATE_COUNT + 1}",
        },
    )


def go(command_param: Dict, sm: smach.StateMachine, person: bool) -> None:
    """
    The possible GO commands are:
        - Go to a location or room and then...

    Person is used to denote whether the robot should go to the location's person
    pose or object pose.

    """
    location_str = ""
    if "location" in command_param:
        target_pose = get_location_pose(command_param["location"], person)
        location_str = command_param["location"]
    elif "room" in command_param:
        target_pose = get_room_pose(command_param["room"])
        location_str = command_param["room"]
    else:
        raise ValueError(
            "Go command received with no location or room in command parameters"
        )
    sm.add(
        f"STATE_{increment_state_count()}",
        Say(text=f"I am going to the {location_str}"),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "aborted": f"STATE_{STATE_COUNT + 1}",
            "preempted": f"STATE_{STATE_COUNT + 1}",
        },
    )
    sm.add(
        f"STATE_{increment_state_count()}",
        GoToLocation(location=target_pose),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": f"STATE_{STATE_COUNT + 1}",
        },
    )


def bring(command_param: Dict, sm: smach.StateMachine) -> None:
    # Bring me a <object> from the <placement location>

    if "location" not in command_param:
        raise ValueError("Bring command received with no location in command paramas")
    if "object" not in command_param:
        raise ValueError("Bring command received with no object in command params")

    location_pose = get_location_pose(command_param["location"], False)
    dest_pose = get_current_pose()

    sm.add(
        f"STATE_{increment_state_count()}",
        Say(
            text=f"I will bring you a {command_param['object']} from the {command_param['location']}"
        ),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT+1}",
            "aborted": f"STATE_{STATE_COUNT + 1}",
            "preempted": f"STATE_{STATE_COUNT + 1}",
        },
    )

    sm.add(
        f"STATE_{increment_state_count()}",
        GoToLocation(location=location_pose),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": f"STATE_{STATE_COUNT + 1}",
        },
    )

    sm.add(
        f"STATE_{increment_state_count()}",
        Say(
            text=f"Please pick up the {command_param['object']} on the {command_param['location']} for me."
        ),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "aborted": f"STATE_{STATE_COUNT + 1}",
            "preempted": f"STATE_{STATE_COUNT + 1}",
        },
    )

    sm.add(
        f"STATE_{increment_state_count()}",
        GoToLocation(
            location=get_location_pose(
                command_param["location"], False, dem_manipulation=True
            ),
        ),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": f"STATE_{STATE_COUNT + 1}",
        },
    )

    sm.add(
        f"STATE_{increment_state_count()}",
        ReceiveObject(object_name=command_param["object"]),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": f"STATE_{STATE_COUNT + 1}",
        },
    )

    sm.add(
        f"STATE_{increment_state_count()}",
        GoToLocation(dest_pose),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": f"STATE_{STATE_COUNT + 1}",
        },
    )

    sm.add(
        f"STATE_{increment_state_count()}",
        HandoverObject(object_name=command_param["object"]),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": f"STATE_{STATE_COUNT + 1}",
        },
    )


def find(command_param: Dict, sm: smach.StateMachine) -> None:
    """
    find a object in the given room
    """

    if "object_category" in command_param or "object" in command_param:
        location_str = ""
        if "location" in command_param:
            waypoints = [get_location_pose(command_param["location"], person=False)]
            location_str = command_param["location"]
            look_points = [get_look_point(command_param["location"])]
        elif "room" in command_param:
            waypoints = get_object_detection_poses(command_param["room"])
            location_str = command_param["room"]
            look_points = get_look_detection_poses(command_param["room"])

        else:
            raise ValueError(
                "Find command with no location or room provided in command parameters"
            )

        if "object_category" in command_param:
            object_str = command_param["object_category"]
            object_filter = OBJECT_CATEGORY_LOCATIONS[command_param["object_category"]]
        elif "object" in command_param:
            object_str = command_param["object"]
            object_filter = [command_param["object"]]

        sm.add(
            f"STATE_{increment_state_count()}",
            Say(text=f"I will go to the {location_str} and find a {object_str}"),
        )
        arena_polygon = rospy.get_param("/gpsr/arena/polygon")
        arena_polygon = Polygon([Point(p[0], p[1], 0.0) for p in arena_polygon])
        sm.add(
            f"STATE_{increment_state_count()}",
            GoFindTheObject(
                model="best.pt",
                waypoints=waypoints,
                filter=object_filter,
                look_points=look_points,
                poly_points=[arena_polygon] * len(waypoints),
            ),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "failed": f"STATE_{STATE_COUNT + 1}",
            },
        )

    elif "gesture" in command_param:
        greet(command_param, sm)
    elif "name" in command_param:
        greet(command_param, sm)


def meet(command_param: Dict, sm: smach.StateMachine) -> None:
    """
    Meet commands can be of the following form:

    - Meet a person at a location and execute another command.
    - Meet a person in a room and execute another command.
    - Meet a person in a rooom then find them in another room.
    """
    greet(command_param, sm)


def count(command_param: Dict, sm: smach.StateMachine) -> None:
    """
    Count commands can be of the following form:

    - Tell me how many objects there are of a given category at a placement location
    - Tell me how many people there are in a given gesture in a given room
    - Tell me how many people there are in a given pose in a given room
    - Tell me how many people there are wearing given clothes in a given room
    """

    people: bool = False
    output_string = "I will count the "
    if "pose" in command_param:
        criteria = "pose"
        criteria_value = command_param["pose"]
        people = True
        output_string += f"{criteria_value} in the "

    elif "gesture" in command_param:
        criteria = "gesture"
        criteria_value = command_param["gesture"]
        if criteria_value.endswith("_persons"):
            criteria_value = criteria_value[: -len("_persons")]
        elif criteria_value.endswith("_people"):
            criteria_value = criteria_value[: -len("_people")]
        people = True
        output_string += f"{criteria_value} in the "

    elif "clothes" in command_param:
        criteria = "clothes"
        raw_val = command_param["clothes"]  # e.g. "blue_t_shirts"
        # 1) normalize underscores/hyphens → spaces, lowercase
        norm = raw_val.replace("_", " ").replace("-", " ").lower()
        # 2) split into words
        parts = norm.split()
        # 3) singularize the garment if it’s plural (“t shirts” → “t shirt”)
        if parts[-1].endswith("s"):
            parts[-1] = parts[-1][:-1]
        # 4) re-join
        criteria_value = " ".join(parts)  # now "blue t shirt"
        people = True
        output_string += f"people wearing {criteria_value} in the "

    elif "object_category" in command_param or "object" in command_param:
        criteria = "object_category"
        # criteria_value = command_param["object_category"]
        criteria_value = command_param.get(
            "object_category", command_param.get("object")
        )
        output_string += f"number of {criteria_value} on the "

    else:
        raise ValueError(
            "Count command received with no pose, gesture, clothes, or object category in command parameters"
        )

    if people:
        if "room" in command_param:
            waypoints: List[Pose] = get_person_detection_poses(command_param["room"])
            polygon: Polygon = get_room_polygon(command_param["room"])
            output_string += command_param["room"]
        else:
            raise ValueError(
                "Count command received pose, gesture, clothes, or object category but no room in command parameters"
            )
        sm.add(
            f"STATE_{increment_state_count()}",
            Say(text=output_string),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "preempted": f"STATE_{STATE_COUNT + 1}",
                "aborted": f"STATE_{STATE_COUNT + 1}",
            },
        )

        sm.add(
            f"STATE_{increment_state_count()}",
            CountPeople(
                waypoints=waypoints,
                polygon=polygon,
                criteria=criteria,
                criteria_value=criteria_value,
            ),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "failed": f"STATE_{STATE_COUNT + 1}",
            },
        )

        sm.add(
            f"STATE_{increment_state_count()}",
            GoToLocation(location=get_current_pose()),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "failed": f"STATE_{STATE_COUNT + 1}",
            },
        )

        sm.add(
            f"STATE_{increment_state_count()}",
            Say(
                format_str="There are {}"
                + criteria_value
                + " people in the "
                + command_param["room"]
            ),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "preempted": f"STATE_{STATE_COUNT + 1}",
                "aborted": f"STATE_{STATE_COUNT + 1}",
            },
            remapping={"placeholders": "people_count"},
        )

    else:
        if not "location" in command_param:
            raise ValueError(
                "Count command with object but no room in command parameters"
            )
        location_pose = get_location_pose(command_param["location"], False)
        location_polygon = get_room_polygon(
            get_location_room((command_param["location"]))
        )
        output_string += command_param["location"]
        """
        sm.add(
            f"STATE_{increment_state_count()}",
            Say(text=output_string),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "preempted": f"STATE_{STATE_COUNT + 1}",
                "aborted": f"STATE_{STATE_COUNT + 1}",
            },
        )
        
        sm.add(
            f"STATE_{increment_state_count()}",
            GoToLocation(location=location_pose),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "failed": f"STATE_{STATE_COUNT + 1}",
            },
        )
        
        sm.add(
            f"STATE_{increment_state_count()}",
            LookToPoint(pointstamped=get_look_point(command_param["location"])),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "aborted": f"STATE_{STATE_COUNT + 1}",
                "timed_out": f"STATE_{STATE_COUNT + 1}",
            },
        )
        """

        sm.add(
            f"STATE_{increment_state_count()}",
            CountObject(
                location_polygon,
                # objects=get_objects_from_category(command_param["object_category"]),
                objects=get_objects_from_category(criteria_value),
                model="best.pt",
            ),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "failed": f"STATE_{STATE_COUNT + 1}",
            },
        )

        sm.add(
            f"STATE_{increment_state_count()}",
            CountObject(
                location_polygon,
                model="best.pt",
                objects=get_objects_from_category(criteria_value),
            ),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "failed": f"STATE_{STATE_COUNT + 1}",
            },
        )

        sm.add(
            f"STATE_{increment_state_count()}",
            GoToLocation(location=get_current_pose()),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "failed": f"STATE_{STATE_COUNT + 1}",
            },
        )

        sm.add(
            f"STATE_{increment_state_count()}",
            Say(
                format_str="There are {} "
                + criteria_value
                + " on the "
                + command_param["location"]
            ),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "aborted": f"STATE_{STATE_COUNT + 1}",
                "preempted": f"STATE_{STATE_COUNT + 1}",
            },
            remapping={"placeholders": "object_count"},
        )

        # --- insert a branch on zero vs nonzero count ---
        if people:
            # --- insert a branch on zero vs nonzero count ---
            @smach.cb_interface(
                input_keys=["people_count"], outcomes=["zero", "nonzero"]
            )
            def _check_zero_cb(userdata):
                return "zero" if userdata.people_count == 0 else "nonzero"

            sm.add(
                f"STATE_{increment_state_count()}",
                smach.CBState(_check_zero_cb),
                transitions={
                    "zero": f"STATE_{STATE_COUNT + 1}_NONE",
                    "nonzero": f"STATE_{STATE_COUNT + 1}_COUNT",
                },
            )

            # zero-case
            sm.add(
                f"STATE_{increment_state_count()}_NONE",
                Say(
                    text=f"There are no people wearing {criteria_value} in the {command_param['room']}"
                ),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "succeeded",
                    "preempted": "succeeded",
                },
            )

            # non-zero case
            sm.add(
                f"STATE_{increment_state_count()}_COUNT",
                Say(
                    format_str=f"There are {{}} people wearing {criteria_value} in the {command_param['room']}"
                ),
                remapping={"placeholders": "people_count"},
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "succeeded",
                    "preempted": "succeeded",
                },
            )
        else:
            # nothing to do here — object_count already handled
            pass


def follow(command_param: Dict, sm: smach.StateMachine, greet_person: bool) -> None:

    if greet_person:
        greet(command_param, sm)

    sm.add(
        f"STATE_{increment_state_count()}",
        Say(text="I will follow you"),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "aborted": f"STATE_{STATE_COUNT + 1}",
            "preempted": f"STATE_{STATE_COUNT + 1}",
        },
    )
    sm.add(
        f"STATE_{increment_state_count()}",
        smach_ros.SimpleActionState(
            "/follow_person",
            FollowAction,
            goal=FollowGoal(),
        ),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "aborted": f"STATE_{STATE_COUNT + 1}",
            "preempted": f"STATE_{STATE_COUNT + 1}",
        },
    )


"""
Build
"""


def build_state_machine(parsed_command: Dict) -> smach.StateMachine:
    """Constructs the parameterized state machine for the GPSR task,
    given the parsed command.

    Args:
        parsed_command (Dict): parsed command.

    Returns:
        smach.StateMachine: paramaterized state machine ready to be executed.
    """
    command_verbs: List[str] = parsed_command["commands"]
    command_params: List[Dict] = parsed_command["command_params"]
    sm = smach.StateMachine(outcomes=["succeeded", "failed"])
    with sm:
        for index, (command_verb, command_param) in enumerate(
            zip(command_verbs, command_params)
        ):
            rospy.loginfo("I am inside!")
            if command_verb == "greet":
                greet(command_param, sm)
            elif command_verb == "talk":
                talk(command_param, sm, greet_person=len(command_verbs) == 1)
            elif command_verb == "guide":
                guide(command_param, sm)
            elif command_verb == "deliver" and len(command_params) > 1:
                deliver(command_param, sm)
            elif command_verb == "deliver" and len(command_params) == 1:
                bring(command_param, sm)
            elif command_verb == "place":
                place(command_param, sm)
            elif command_verb == "take":
                if "object" in command_param or "object_category" in command_param:
                    take(command_param, sm)
                else:
                    guide(command_param, sm)
            elif command_verb == "answer":
                answer(command_param, sm, greet_person=len(command_verbs) == 1)
            elif command_verb == "go":
                person = not any(
                    [
                        param in ["object", "object_category"]
                        for param in command_params[index:]
                    ]
                )
                go(command_param, sm, person)
            elif command_verb == "find":
                find(command_param, sm)
            elif command_verb == "meet":
                meet(command_param, sm)
            elif command_verb == "count":
                count(command_param, sm)
            elif command_verb == "follow":
                pass
                # follow(command_param, sm, greet_person=len(command_verbs) == 1)
            else:
                raise ValueError(f"Unrecognised command verb: {command_verb}")

        @smach.cb_interface(outcomes=["succeeded"])
        def terminal_cb(_):
            return "succeeded"

        sm.add(
            f"STATE_{increment_state_count()}",
            smach.CBState(terminal_cb),
            transitions={"succeeded": "succeeded"},
        )

    rospy.loginfo(f"State machine: {sm}")
    return sm
