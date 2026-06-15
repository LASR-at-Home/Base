import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    load_motions = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("skills"),
                "launch",
                "load_motions.launch.py",
            )
        )
    )

    bodypix_service = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lasr_vision_bodypix"),
                "launch",
                "bodypix_launch.py",
            )
        )
    )

    yolo_service = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lasr_vision_yolo"),
                "launch",
                "service_launch.xml",
            )
        )
    )
    llm_service = Node(
        package="lasr_llm",
        executable="restaurant_service",
        name="restaurant_query_llm_service",
        output="screen",
    )

    speech_recognition = Node(
        package="lasr_speech_recognition_whisper",
        executable="transcribe_speech",
        name="transcribe_speech",
        output="screen",
    )

    restaurant = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="restaurant",
                executable="sm",
                name="restaurant",
                parameters=[
                    os.path.join(
                        get_package_share_directory("restaurant"), "config", "lab.yaml"
                    )
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            load_motions,
            yolo_service,
            restaurant,
            llm_service,
            speech_recognition,
        ]
    )
