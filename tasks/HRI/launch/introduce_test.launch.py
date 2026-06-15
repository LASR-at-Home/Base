import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():

    # YOLO detection service
    yolo_service = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lasr_vision_yolo"),
                "launch",
                "service_launch.xml",
            )
        )
    )

    # ReID service
    reid_service = Node(
        package="lasr_vision_reid",
        executable="service",
        name="lasr_vision_reid",
        output="screen",
    )

    # TTS engine
    tts_engine = Node(
        package="tts_engine",
        executable="tts_engine",
        name="tts_engine",
        output="screen",
    )

    # Configure TTS after 15 seconds to give it time to start
    tts_configure = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/tts_engine", "configure"],
                output="screen",
            )
        ],
    )

    # Activate TTS after configure
    tts_activate = TimerAction(
        period=18.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/tts_engine", "activate"],
                output="screen",
            )
        ],
    )

    # Introduce test node — delayed to give all services time to start
    introduce_test = TimerAction(
        period=25.0,
        actions=[
            Node(
                package="HRI",
                executable="introduce",
                name="introduce_test_node",
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            yolo_service,
            reid_service,
            tts_engine,
            tts_configure,
            tts_activate,
            introduce_test,
        ]
    )
