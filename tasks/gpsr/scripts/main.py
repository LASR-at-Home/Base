#!/usr/bin/env python3
import os
import smach
import rospy
import rospkg
import sys
from typing import Dict
from gpsr.load_known_data import GPSRDataLoader
from gpsr.state_machine_factory import build_state_machine
from gpsr.regex_command_parser import Configuration
from gpsr.states import CommandParserStateMachine

import actionlib
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pal_interaction_msgs.msg import TtsGoal, TtsAction


def load_gpsr_configuration() -> Configuration:
    gpsr_data_dir = os.path.join(rospkg.RosPack().get_path("gpsr"), "data", "mock_data")
    """Loads the configuration for the GPSR command parser"""
    data_loader = GPSRDataLoader(data_dir=gpsr_data_dir)
    gpsr_known_data: Dict = data_loader.load_data()
    config = Configuration(
        {
            "person_names": gpsr_known_data["names"],
            "location_names": gpsr_known_data["non_placeable_locations"],
            "placement_location_names": gpsr_known_data["placeable_locations"],
            "room_names": gpsr_known_data["rooms"],
            "object_names": gpsr_known_data["objects"],
            "object_categories_plural": gpsr_known_data["categories_plural"],
            "object_categories_singular": gpsr_known_data["categories_singular"],
        }
    )
    return config


def _move_base(client: actionlib.SimpleActionClient, pose: PoseStamped) -> MoveBaseGoal:
    goal: MoveBaseGoal = MoveBaseGoal()
    goal.target_pose = pose
    client.send_goal(goal)

    return goal


def _tts(client: actionlib.SimpleActionClient, text: str) -> None:
    tts_goal: TtsGoal = TtsGoal()
    tts_goal.rawtext.text = text
    tts_goal.rawtext.lang_id = "en_GB"
    client.send_goal(tts_goal)


def main() -> None:
    instruction_pose_param: Dict = rospy.get_param("gpsr/arena/start_pose")
    instruction_pose: PoseStamped = PoseStamped(
        pose=Pose(
            position=Point(**instruction_pose_param["position"]),
            orientation=Quaternion(**instruction_pose_param["orientation"]),
        )
    )
    N_COMMANDS: int = 3
    config = load_gpsr_configuration()
    move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    move_base_client.wait_for_server()
    tts_client = actionlib.SimpleActionClient("tts", TtsAction)
    tts_client.wait_for_server()
    for i in range(N_COMMANDS):
        rospy.loginfo(f"Command {i + 1}")
        _tts(tts_client, "I am going to the instruction point to receive a command")
        _move_base(move_base_client, instruction_pose)
        command_parser_sm = CommandParserStateMachine(data_config=config)
        command_parser_sm.execute()
        parsed_command: Dict = command_parser_sm.userdata.parsed_command
        rospy.loginfo(f"Parsed command: {parsed_command}")
        sm = build_state_machine(parsed_command)
        sm.execute()
    _tts(tts_client, "I am done")


if __name__ == "__main__":
    rospy.init_node("gpsr_main")
    main()
    rospy.spin()
