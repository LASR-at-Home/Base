#!/usr/bin/env python3
import rospy
import smach
from smach_ros import ServiceState
from typing import Dict, List
from lasr_skills import GoToLocation, FindNamedPerson
from gpsr.states import Talk

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
                if "name" in command_param:
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
                        FindNamedPerson(
                            name=command_param["name"], location_param=location_param
                        ),
                        transitions={
                            "succeeded": f"STATE_{STATE_COUNT + 1}",
                            "failed": "failed",
                        },
                    )
                elif "clothes" in command_param:
                    pass
                else:
                    raise ValueError(
                        "Greet command received with no name or clothes in command parameters"
                    )
            elif command_verb == "talk":
                if "gesture" in command_param:
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
