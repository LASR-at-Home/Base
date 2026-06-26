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

    load_motions = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("skills"),
                "launch",
                "load_motions.launch.py",
            )
        )
    )

    whisper_server = Node(
        package="lasr_speech_recognition_whisper",
        executable="transcribe_microphone_server",
        name="whisper_mic_server",
        output="screen",
    )

    state_machine = TimerAction(
        period=30.0,
        actions=[
            Node(
                package="GPSR",
                executable="sm",
                name="gpsr",
                output="screen",
                parameters=[
                    params,
                    {"input_mode": input_mode},
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_mode",
                default_value="mic",
                description='Command input source: "keyboard" or "mic"',
            ),
            load_motions,
            whisper_server,
            state_machine,
        ]
    )
