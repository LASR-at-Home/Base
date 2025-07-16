#!/usr/bin/env python3
import rospy
import itertools
import re
from tqdm import tqdm
from gpsr.load_known_data import GPSRDataLoader


def main():
    command_template_path = "/home/mattbarker/robot_club/lasr_ws/src/Base/tasks/gpsr/data/command_data/placeholder_commands.txt"
    with open(command_template_path, "r") as file:
        commands = file.readlines()
    commands = [command.strip().lower() for command in commands if command.strip()]
    template_names = ["olivia", "noah"]
    template_locations = ["chair", "desk", "office chair"]
    template_placement_locations = ["chair", "desk"]
    template_rooms = ["bedroom", "kitchen"]
    template_objects = ["crisps", "cracker"]
    template_category_singular = ["snack"]
    template_category_plural = ["snacks"]

    template_commands = []
    for command in tqdm(commands):
        template_command = command
        for name in template_names:
            if name in command:
                template_command = template_command.replace(name, "{names}")
        for location in template_locations:
            if location in command:
                template_command = template_command.replace(
                    location, "{non_placeable_locations}"
                )
        for placement_location in template_placement_locations:
            if placement_location in command:
                template_command = template_command.replace(
                    placement_location, "{placeable_locations}"
                )
        for room in template_rooms:
            if room in command:
                template_command = template_command.replace(room, "{rooms}")
        for obj in template_objects:
            if obj in command:
                template_command = template_command.replace(obj, "{objects}")
        for category_singular in template_category_singular:
            if category_singular in command:
                template_command = template_command.replace(
                    category_singular, "{categories_singular}"
                )
        for category_plural in template_category_plural:
            if category_plural in command:
                template_command = template_command.replace(
                    category_plural, "{categories_plural}"
                )
        if template_command not in template_commands:
            template_commands.append(template_command)

    with open(
        "/home/mattbarker/robot_club/lasr_ws/src/Base/tasks/gpsr/data/command_data/placeholder_commands_parsed.txt",
        "w",
    ) as file:
        for command in template_commands:
            file.write(f"{command}\n")

    loader = GPSRDataLoader(
        data_dir="/home/mattbarker/robot_club/lasr_ws/src/Base/tasks/gpsr/data/salvador_data"
    )
    data = loader.load_data()
    print(f"Loaded GPSR data: {data}")

    # Load template commands
    template_path = "/home/mattbarker/robot_club/lasr_ws/src/Base/tasks/gpsr/data/command_data/placeholder_commands_parsed.txt"
    with open(template_path, "r") as file:
        template_commands = file.readlines()
    template_commands = [
        command.strip().lower() for command in template_commands if command.strip()
    ]

    def expand_template(template, data):
        # Extract placeholders in order (duplicates allowed)
        placeholders = re.findall(r"{(.*?)}", template)
        # print(placeholders)
        value_lists = [data[p] for p in placeholders]
        # print(value_lists)
        combinations = itertools.product(*value_lists)

        results = []
        for combo in combinations:
            temp = template
            for val in combo:
                temp = re.sub(r"\{.*?\}", val, count=1, string=temp)
            results.append(temp)
        return results

    all_commands = []
    for template in template_commands:
        expanded = expand_template(template, data)
        all_commands.extend(expanded)
        rospy.loginfo(f"Generated {len(all_commands)} commands.")

    with open(
        "/home/mattbarker/robot_club/lasr_ws/src/Base/tasks/gpsr/data/command_data/all_commands_robocup_2025.txt",
        "w",
    ) as file:
        for command in all_commands:
            file.write(f"{command}\n")


if __name__ == "__main__":
    rospy.init_node("gpsr_main")
    main()
    rospy.spin()
