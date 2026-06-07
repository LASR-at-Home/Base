import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    input_mode = LaunchConfiguration("input_mode")

    pkg_sim = get_package_share_directory("simulation")
    pkg_gpsr = get_package_share_directory("GPSR")

    params = os.path.join(pkg_gpsr, "config", "params.yaml")

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_sim, "launch", "nav.launch.py"))
    )

    rviz = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", os.path.join(pkg_sim, "config", "mapping.rviz")],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        ],
    )

    map_server_deactivate = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/map_server", "deactivate"],
                output="screen",
            )
        ],
    )
    map_server_cleanup = TimerAction(
        period=21.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/map_server", "cleanup"],
                output="screen",
            )
        ],
    )
    map_server_configure = TimerAction(
        period=22.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/map_server", "configure"],
                output="screen",
            )
        ],
    )
    map_server_activate = TimerAction(
        period=23.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/map_server", "activate"],
                output="screen",
            )
        ],
    )

    initial_pose = TimerAction(
        period=24.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "--once",
                    "/initialpose",
                    "geometry_msgs/msg/PoseWithCovarianceStamped",
                    (
                        '{"header": {"frame_id": "map"}, "pose": {"pose": {'
                        '"position": {"x": 9.151, "y": -6.340, "z": 0.0}, '
                        f'"orientation": {{"x": 0.0, "y": 0.0, "z": {round(math.sin(2.204 / 2), 6)}, "w": {round(math.cos(2.204 / 2), 6)}}}'
                        '}, "covariance": [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}'
                    ),
                ],
                output="screen",
            )
        ],
    )

    whisper_server = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "lasr_speech_recognition_whisper",
            "transcribe_microphone_server",
            "--no_warmup",
            "--energy_threshold",
            "6500",
            "--device",
            "cpu",
        ],
        output="screen",
        condition=IfCondition(
            PythonExpression(
                ["'", input_mode, "' == 'mic' or '", input_mode, "' == 'microphone'"]
            )
        ),
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
                default_value="keyboard",
                description='Command input source: "keyboard" or "mic"',
            ),
            navigation,
            rviz,
            map_server_deactivate,
            map_server_cleanup,
            map_server_configure,
            map_server_activate,
            initial_pose,
            whisper_server,
            state_machine,
        ]
    )
