#!/usr/bin/env python3
import rospy
import smach
from smach_ros import ServiceState
from typing import Dict, List
from lasr_skills import GoToLocation, FindPerson
from gpsr.states import Talk

from geometry_msgs.msg import Pose, Point, Quaternion, Polygon

STATE_COUNT = 0


def increment_state_count() -> int:
    global STATE_COUNT
    STATE_COUNT += 1
    return STATE_COUNT


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
        for command_verb, command_param in zip(command_verbs, command_params):
            if command_verb == "greet":
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
                    [
                        Point(**p)
                        for p in rospy.get_param(location_param)["polygon"]["points"]
                    ]
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

            elif command_verb == "talk":
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

    rospy.loginfo(f"State machine: {sm}")
    return sm
