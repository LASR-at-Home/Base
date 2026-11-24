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

from GPSR.world import vocabulary_prompt


def generate_launch_description():
    input_mode = LaunchConfiguration("input_mode")

    pkg_gpsr = get_package_share_directory("GPSR")
    params = os.path.join(pkg_gpsr, "config", "params.yaml")

    sim = DeclareLaunchArgument("sim", default_value="False")
    yolo_service = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lasr_vision_yolo"),
                "launch",
                "service_launch.xml",
            )
        ),
        launch_arguments={
            "sim": LaunchConfiguration("sim"),
            # seg model for Detect3D, pose model for the gesture service
            "preload": "['yolo11n-seg.pt', 'yolo11n-pose.pt']",
        }.items(),
    )

    # ADded new
    gesture_service = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lasr_vision_gesture"),
                "launch",
                "gesture_service.launch.xml",
            )
        )
    )

    # Added new
    vlm = Node(
        package="lasr_vlm",
        executable="vlm_service",
        name="lasr_vlm_service",
        output="screen",
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
                # people / objects / locations from config/*.yaml
                "initial_prompt": vocabulary_prompt(),
                "model": LaunchConfiguration("whisper_model"),
                "mic_device": LaunchConfiguration("mic_device"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "whisper_model",
                default_value="small.en",
                description="Whisper model: small.en (fast) or medium.en (more accurate, more GPU memory)",
            ),
            DeclareLaunchArgument(
                "mic_device",
                default_value="default",
                description="Microphone: index or part of its name (see list_microphones.py)",
            ),
            DeclareLaunchArgument(
                "input_mode",
                default_value="mic",
                description='Command input source: "keyboard" or "mic"',
            ),
            sim,
            whisper_server,
            load_motions,
            yolo_service,
            gesture_service,  # find_person / count_people by gesture
            vlm,  # get_person_info / find_person by clothes
        ]
    )


# ros2 run GPSR sm --ros-args --params-file src/Base/tasks/GPSR/config/params.yaml
