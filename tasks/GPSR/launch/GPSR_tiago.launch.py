import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_mode = LaunchConfiguration("input_mode")

    pkg_gpsr = get_package_share_directory("GPSR")

    params = os.path.join(pkg_gpsr, "config", "params.yaml")

    yolo_service = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lasr_vision_yolo"),
                "launch",
                "service_launch.xml",
            )
        )
    )

    load_motions = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("skills"),
                "launch",
                "load_motions.launch.py",
            )
        )
    )
    robot_ui = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_ui"),
                "launch",
                "robot_ui.launch.py",
            )
        )
    )

    whisper_server = Node(
        package="lasr_speech_recognition_whisper",
        executable="transcribe_microphone_server",
        name="whisper_mic_server",
        output="screen",
        parameters=[
            {
                "initial_prompt": "adel, Angel, axel, charlie, jane, jules, morgan, paris, robin, simone, white_shirt, grey_shirt, blue_shirt, black_shirt, hand_towel, rubiks_cube, pringles, seaweed, apple, peach, mangostane, lemon, yellow_bellpepper, red_bellpepper, instant_noodles, cornflakes, coke, red_bull, milk, soju, pepsi, dishwasher_tab, sponge, toothpaste, cup, spoon, plate, knife, fork, bowl, bedroom, kitchen, laundry, living room, instruction point, laundry table, washing machine, shelf, laundry trash bin, bed, bedside table, coat rack, tv stand, sofa, coffee table, cabinet, refrigerator, counter, sink, cooking table, dishwasher, kitchen trash bin, dinner table, entrance, exit"
            }  # Update the initial prompt with the list of objects and locations
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_mode",
                default_value="mic",
                description='Command input source: "keyboard" or "mic"',
            ),
            whisper_server,
            load_motions,
            yolo_service,
        ]
    )


# ros2 run GPSR sm --ros-args --params-file src/Base/tasks/GPSR/config/params.yaml
