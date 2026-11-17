import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from GPSR.world import vocabulary_prompt


def generate_launch_description():
    input_mode = LaunchConfiguration("input_mode")

    pkg_gpsr = get_package_share_directory("GPSR")

    params = os.path.join(pkg_gpsr, "config", "params.yaml")

    gpsr_service_node = Node(
        package="GPSR",
        executable="service",
        name="gpsr",  # must match the top-level key in params.yaml
        output="screen",
        parameters=[params],
    )
    delayed_gpsr_service = TimerAction(
        period=5.0, actions=[gpsr_service_node]  # Delay in seconds (adjust as needed)
    )

    sim = DeclareLaunchArgument("sim", default_value="False")
    yolo_service = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lasr_vision_yolo"),
                "launch",
                "service_launch.xml",
            )
        ),
        launch_arguments={"sim": LaunchConfiguration("sim")}.items(),
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

    ollama = ExecuteProcess(cmd=["ollama", "serve"], output="log")

    whisper_server = Node(
        package="lasr_speech_recognition_whisper",
        executable="transcribe_microphone_server",
        name="whisper_mic_server",
        output="screen",
        parameters=[
            {
                "initial_prompt": vocabulary_prompt()
            }  # people / objects / locations from config/*.yaml
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_mode",
                default_value="mic",
                description='Command input source: "keyboard" or "mic"',
            ),
            sim,
            whisper_server,
            load_motions,
            yolo_service,
            ollama,
            delayed_gpsr_service,
        ]
    )


# ros2 run GPSR service --ros-args --params-file src/Base/tasks/GPSR/config/params.yaml
