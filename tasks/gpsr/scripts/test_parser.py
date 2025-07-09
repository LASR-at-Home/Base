#!/usr/bin/env python3
import argparse
import smach
import rospy
from typing import Dict
from tqdm import tqdm

from gpsr.load_known_data import GPSRDataLoader
from gpsr.regex_command_parser import Configuration, gpsr_compile_and_parse
from lasr_skills import AskAndListen, Say

if __name__ == "__main__":
    rospy.init_node("gpsr_command_parser")
    data_loader = GPSRDataLoader(
        data_dir="/home/mattbarker/robot_club/lasr_ws/src/Base/tasks/gpsr/data/mock_data"
    )
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
    command_path = "/home/mattbarker/robot_club/lasr_ws/src/Base/tasks/gpsr/data/command_data/placeholder_commands.txt"
    # Read commands
    with open(command_path, "r") as file:
        commands = file.readlines()
    commands = [commands.strip().lower() for commands in commands if commands.strip()]
    failed_commands = []

    gpsr_compile_and_parse(config, commands)

    # print(f"Loaded configuration: {config}")

    # for index, command in tqdm(enumerate(commands)):
    #     try:
    #         parsed_command = gpsr_compile_and_parse(config, command)
    #     except:
    #         failed_commands.append(command)

    #     print(
    #         f"[INDEX] [{index}] Failed command Percentage: {(len(failed_commands) / (index + 1)) * 100:.2f}%"
    #     )
    #     if index % 1000 == 0:
    #         with open("failed_commands.txt", "w") as f:
    #             for failed_command in failed_commands:
    #                 f.write(f"{failed_command}\n")

    # print(
    #     f"Failed command percentage: {len(failed_commands) / len(commands) * 100:.2f}%"
    # )

    rospy.spin()
