#!/usr/bin/env python3
import smach
import rospy
import sys
from typing import Dict
from gpsr.load_known_data import GPSRDataLoader
from gpsr.regex_command_parser import Configuration
from gpsr.states import CommandParserStateMachine


def load_gpsr_configuration() -> Configuration:
    gpsr_data_dir = sys.argv[1]
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


def main():
    config = load_gpsr_configuration()
    command_parser_sm = CommandParserStateMachine(data_config=config)
    command_parser_sm.execute()
    parsed_command: Dict = command_parser_sm.userdata.parsed_command
    rospy.loginfo(f"Parsed command: {parsed_command}")


if __name__ == "__main__":
    rospy.init_node("gpsr_main")
    main()
    rospy.spin()
