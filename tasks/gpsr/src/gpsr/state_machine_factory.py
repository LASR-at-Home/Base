#!/usr/bin/env python3
import rospy
import smach
from typing import Dict, List
from lasr_skills import GoToLocation, FindNamedPerson, FindGesturePerson  # type: ignore
from gpsr.states import Talk


class GPSRStateMachineFactory:
    def __init__(self, parsed_command: Dict):
        """Stores the parsed command, initializes the state count,
        and initalises the state machine.

        Args:
            parsed_command (Dict): parsed command output by the
            command parser.
        """
        self.state_count: int = 0
        self.parsed_command: dict = parsed_command
        self.sm = smach.StateMachine(outcomes=["succeeded", "failed"])

    def _increment_state_count(self):
        self.state_count += 1
        return self.state_count

    def _handle_take_command(self, command_param: Dict):
        raise NotImplementedError("Take command not implemented")

    def _handle_place_command(self, command_param: Dict):
        raise NotImplementedError("Place command not implemented")

    def _handle_deliver_command(self, command_param: Dict):
        raise NotImplementedError("Deliver command not implemented")

    def _handle_bring_command(self, command_param: Dict):
        raise NotImplementedError("Bring command not implemented")

    def _handle_go_command(self, command_param: Dict):
        raise NotImplementedError("Go command not implemented")

    def _handle_find_command(self, command_param: Dict):
        raise NotImplementedError("Find command not implemented")

    def _handle_talk_command(self, command_param: Dict):
        if "gesture" in command_param:
            if not "location" in command_param:
                raise ValueError(
                    "Talk command with gesture but no room in command parameters"
                )
            location_param_room = f"/gpsr/arena/rooms/{command_param['location']}"
            self.sm.add(
                f"STATE_{self._increment_state_count()}",
                FindGesturePerson(
                    location_param=location_param_room, gesture=command_param["gesture"]
                ),
                transitions={
                    "succeeded": f"STATE_{self.state_count + 1}",
                    "failed": "failed",
                },
            )
        if "talk" in command_param:
            self.sm.add(
                f"STATE_{self._increment_state_count()}",
                Talk(talk_phrase=command_param["talk"]),
                transitions={
                    "succeeded": f"STATE_{self.state_count + 1}",
                    "failed": "failed",
                },
            )
        else:
            raise ValueError(
                "Talk command received with no text or gesture in command parameters"
            )

    def _handle_answer_command(self, command_param: Dict):
        raise NotImplementedError("Answer command not implemented")

    def _handle_meet_command(self, command_param: Dict):
        raise NotImplementedError("Meet command not implemented")

    def _handle_tell_command(self, command_param: Dict):
        raise NotImplementedError("Tell command not implemented")

    def _handle_greet_command(self, command_param: Dict):
        if "name" in command_param:
            location_param_room = f"/gpsr/arena/rooms/{command_param['location']}"
            location_param_pose = f"{location_param_room}/pose"
            self.sm.add(
                f"STATE_{self._increment_state_count()}",
                GoToLocation(location_param=location_param_pose),
                transitions={
                    "succeeded": f"STATE_{self.state_count + 1}",
                    "failed": "failed",
                },
            )
            self.sm.add(
                f"STATE_{self._increment_state_count()}",
                FindNamedPerson(
                    name=command_param["name"],
                    location_param=location_param_room,
                ),
                transitions={
                    "succeeded": f"STATE_{self.state_count + 1}",
                    "failed": "failed",
                },
            )
        elif "clothes" in command_param:
            raise NotImplementedError("Greet command with clothes not implemented")
        else:
            raise ValueError(
                "Greet command received with no name or clothes in command parameters"
            )

    def _handle_count_command(self, command_params: Dict):
        raise NotImplementedError("Count command not implemented")

    def _handle_follow_command(self, command_params: Dict):
        raise NotImplementedError("Follow command not implemented")

    def _handle_guide_command(self, command_params: Dict):
        raise NotImplementedError("Guide command not implemented")

    def build_state_machine(self) -> smach.StateMachine:
        with self.sm:
            command_verbs: List[str] = self.parsed_command["commands"]
            command_params: List[Dict] = self.parsed_command["command_params"]
            for command_verb, command_param in zip(command_verbs, command_params):
                if command_verb == "take":
                    self._handle_take_command(command_param)
                elif command_verb == "place":
                    self._handle_place_command(command_param)
                elif command_verb == "deliver":
                    self._handle_deliver_command(command_param)
                elif command_verb == "bring":
                    self._handle_bring_command(command_param)
                elif command_verb == "go":
                    self._handle_go_command(command_param)
                elif command_verb == "find":
                    self._handle_find_command(command_param)
                elif command_verb == "talk":
                    self._handle_talk_command(command_param)
                elif command_verb == "answer":
                    self._handle_answer_command(command_param)
                elif command_verb == "meet":
                    self._handle_meet_command(command_param)
                elif command_verb == "tell":
                    self._handle_tell_command(command_param)
                elif command_verb == "greet":
                    self._handle_greet_command(command_param)
                elif command_verb == "count":
                    self._handle_count_command(command_param)
                elif command_verb == "follow":
                    self._handle_follow_command(command_param)
                elif command_verb == "guide":
                    self._handle_guide_command(command_param)
                else:
                    raise ValueError(f"Invalid command verb: {command_verb}")

            self.sm.add(
                f"STATE_{self.state_count+1}",
                Talk(talk_phrase="I have completed the task."),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )

        return self.sm
