#!/usr/bin/env python3
import rospy
import smach
from smach_ros import ServiceState
from typing import Dict, List
from lasr_skills import (
    GoToLocation,
    FindPerson,
    Guide,
    HandoverObject,
    Say,
    ReceiveObject,
)
from gpsr.states import Talk, QuestionAnswer, GoFindTheObject, ObjectComparison

from geometry_msgs.msg import Pose, Point, Quaternion, Polygon

STATE_COUNT = 0
from lasr_skills import GoToLocation, FindNamedPerson, FindGesturePerson  # type: ignore


def increment_state_count() -> int:
    global STATE_COUNT
    STATE_COUNT += 1
    return STATE_COUNT


def get_location_room(location: str) -> str:
    for room in rospy.get_param("/gpsr/arena/rooms"):
        if location in room["beacons"]:
            return room
    raise ValueError(f"Location {location} not found in the arena")


def get_location_pose(location: str, person: bool) -> Pose:
    location_room = get_location_room(location)
    if person:
        location_pose: Dict = rospy.get_param(
            f"/gpsr/arena/rooms/{location_room}/beacons/{location}/person_detection_pose"
        )
    else:
        location_pose: Dict = rospy.get_param(
            f"/gpsr/arena/rooms/{location_room}/beacons/{location}/object_detection_pose"
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
        [Point(**p) for p in rospy.get_param(f"/gpsr/arena/rooms/{room}/room_polygon")]
    )


def greet(command_param: Dict, sm: smach.StateMachine) -> None:
    target_pose = get_room_pose(command_param["room"])
    sm.add(
        f"STATE_{increment_state_count()}",
        GoToLocation(location=target_pose),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": "failed",
        },
    )

    waypoints: List[Pose] = get_person_detection_poses(command_param["room"])
    polygon: Polygon = get_room_polygon(command_param["room"])

    if "name" in command_param:
        criteria = "name"
        criteria_value = command_param["name"]
    elif "clothes" in command_param:
        criteria = "clothes"
        criteria_value = command_param["clothes"]
    elif "gesture" in command_param:
        criteria = "gesture"
        criteria_value = command_param["gesture"]
    elif "pose" in command_param:
        criteria = "pose"
        criteria_value = command_param["pose"]
    else:
        raise ValueError(
            "Greet command received with no name, clothes, gesture, or pose in command parameters"
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
            "failed": "failed",
        },
    )


def talk(command_param: Dict, sm: smach.StateMachine) -> None:
    if "gesture" in command_param:
        # TODO:
        pass
    elif "talk" in command_param:
        sm.add(
            f"STATE_{increment_state_count()}",
            Talk(command_param["talk"]),
            transitions={"succeeded": "succeeded", "failed": "failed"},
        )
    else:
        raise ValueError(
            "Talk command received with no gesture or talk in command parameters"
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

    # TODO: need to handle rooms or location
    location_param = f"/gpsr/arena/rooms/{command_param['location']}"
    location_name = command_param["location"]
    location_pose = Pose(
        position=Point(**rospy.get_param(f"{location_param}/pose/position")),
        orientation=Quaternion(**rospy.get_param(f"{location_param}/pose/orientation")),
    )

    sm.add(
        f"STATE_{increment_state_count()}",
        Guide(location_name=location_name, location_pose=location_pose),
        transitions={"succeeded": "succeeded", "failed": "failed"},
    )


def deliver(command_param: Dict, sm: smach.StateMachine) -> None:
    location_param = f"/gpsr/arena/rooms/{command_param['location']}"
    sm.add(
        f"STATE_{increment_state_count()}",
        GoToLocation(location_param=f"{location_param}/pose"),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": "failed",
        },
    )

    waypoints: List[Pose] = [
        Pose(
            position=Point(**wp["position"]),
            orientation=Quaternion(**wp["orientation"]),
        )
        for wp in rospy.get_param(location_param)["waypoints"]
    ]

    polygon = Polygon(
        [Point(**p) for p in rospy.get_param(location_param)["polygon"]["points"]]
    )

    if "name" in command_param:
        criteria = "name"
        criteria_value = command_param["name"]
    elif "clothes" in command_param:
        criteria = "clothes"
        criteria_value = command_param["clothes"]
    elif "gesture" in command_param:
        criteria = "gesture"
        criteria_value = command_param["gesture"]
    elif "pose" in command_param:
        criteria = "pose"
        criteria_value = command_param["pose"]
    else:
        raise ValueError(
            "Deliver command received with no name, clothes, gesture, or pose in command parameters"
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
            "failed": "failed",
        },
    )
    sm.add(
        f"STATE_{increment_state_count()}",
        HandoverObject(object_name="object"),  # TODO: pass object name
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": "failed",
        },
    )


def place(command_param: Dict, sm: smach.StateMachine) -> None:
    location_param = f"/gpsr/arena/rooms/{command_param['location']}"
    sm.add(
        f"STATE_{increment_state_count()}",
        GoToLocation(location_param=f"{location_param}/pose"),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": "failed",
        },
    )

    sm.add(
        f"STATE_{increment_state_count()}",
        HandoverObject(object_name="object"),  # TODO: pass object name
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": "failed",
        },
    )

    sm.add(
        f"STATE_{increment_state_count()}",
        Say(text=f"Please place the object on the {command_param['location']}"),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "aborted": "failed",
            "preempted": "failed",
        },
    )


def take(command_param: Dict, sm: smach.StateMachine) -> None:
    # TODO: need to redo this to handle placement locations
    location_param = f"/gpsr/arena/rooms/{command_param['location']}"
    sm.add(
        f"STATE_{increment_state_count()}",
        GoToLocation(location_param=f"{location_param}/pose"),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": "failed",
        },
    )

    sm.add(
        f"STATE_{increment_state_count()}",
        Say(
            text=f"Please pick up the {command_param['object_category']} on the {command_param['location']} for me."
        ),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "aborted": "failed",
            "preempted": "failed",
        },
    )

    sm.add(
        f"STATE_{increment_state_count()}",
        ReceiveObject(object_name=command_param["object_category"]),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": "failed",
        },
    )


def answer(command_param: Dict, sm: smach.StateMachine) -> None:
    sm.add(
        f"STATE_{increment_state_count()}",
        # TODO: pass index_path, txt_path, xml_path
        QuestionAnswer(
            index_path="index.txt",
            txt_path="txt",
            xml_path="xml",
            k=1,
        ),
        transitions={
            "succeeded": f"STATE_{STATE_COUNT + 1}",
            "failed": "failed",
        },
    )


def go(command_param: Dict, sm: smach.StateMachine, person: bool) -> None:
    """
    The possible GO commands are:
        - Go to a location or room and then...

    Person is used to denote whether the robot should go to the location's person
    pose or object pose.

    """
    if "location" in command_param:
        target_pose = get_location_pose(command_param["location"], person)
    elif "room" in command_param:
        target_pose = get_room_pose(command_param["room"])
    else:
        raise ValueError(
            "Go command received with no location or room in command parameters"
        )

    sm.add(
        f"STATE_{increment_state_count()}",
        GoToLocation(location=target_pose),
        transitions={
            "succeeded": "succeeded",
            "failed": "failed",
        },
    )


def bring(command_param: Dict, sm: smach.StateMachine) -> None:
    pass


def find(command_param: Dict, sm: smach.StateMachine) -> None:
    # find a object in the given room
    if "object_category" in command_param:
        if not "location" in command_param:
            raise ValueError(
                "find command with object but no room in command parameters"
            )
        location_param_room = f"/gpsr/arena/rooms/{command_param['location']}"
        sm.add(
            f"STATE_{increment_state_count()}",
            GoFindTheObject(
                location_param=location_param_room, object=command_param["object_category"]
            ),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "failed": "failed",
            },
        )
    raise NotImplementedError("Find command not implemented")


def meet(command_param: Dict, sm: smach.StateMachine) -> None:
    pass


def tell(command_param: Dict, sm: smach.StateMachine) -> None:
    pass


def count(command_param: Dict, sm: smach.StateMachine) -> None:
    # Count the number of a category of objects at a given placement location
    if "object_category" in command_param:
        if not "location" in command_param:
            raise ValueError(
                "Count command with object but no room in command parameters"
            )
        location_param_room = f"/gpsr/arena/rooms/{command_param['location']}"
        sm.add(
            f"STATE_{increment_state_count()}",
            GoFindTheObject(
                location_param=location_param_room, object=command_param["object_category"]
            ),
            transitions={
                "succeeded": f"STATE_{STATE_COUNT + 1}",
                "failed": "failed",
            },
        )


    raise NotImplementedError("Count command not implemented")

    pass


def follow(command_param: Dict, sm: smach.StateMachine) -> None:
    pass


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
            if command_verb == "greet":
                greet(command_param, sm)
            elif command_verb == "talk":
                talk(command_param, sm)
            elif command_verb == "guide":
                guide(command_param, sm)
            elif command_verb == "deliver":
                deliver(command_param, sm)
            elif command_verb == "place":
                place(command_param, sm)
            elif command_verb == "take":
                take(command_param, sm)
            elif command_verb == "answer":
                answer(command_param, sm)
            elif command_verb == "go":
                person = not any(
                    [
                        param in ["object", "object_category"]
                        for param in command_params[index:]
                    ]
                )
                go(command_param, sm, person)
            elif command_verb == "bring":
                bring(command_param, sm)
            elif command_verb == "find":
                find(command_param, sm)
            elif command_verb == "meet":
                meet(command_param, sm)
            elif command_verb == "tell":
                tell(command_param, sm)
            elif command_verb == "count":
                count(command_param, sm)
            elif command_verb == "follow":
                follow(command_param, sm)
            else:
                raise ValueError(f"Unrecognised command verb: {command_verb}")

    rospy.loginfo(f"State machine: {sm}")
    return sm
