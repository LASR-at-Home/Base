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

    yolo_service = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lasr_vision_yolo"),
                "launch",
                "service_launch.xml",
            )
        )
    )

    vision_clip = Node(
        package="lasr_vision_clip",
        executable="vqa",
        name="lasr_vision_clip_service",
        output="screen",
    )

    reid_service = Node(
        package="lasr_vision_reid",
        executable="service",
        name="lasr_vision_reid",
        output="screen",
    )

    eye_tracker = Node(
        package="lasr_vision_eye_tracker",
        executable="eye_tracker_action_server",
        name="eye_tracker_action_server",
        output="screen",
    )

    state_machine = Node(
        package="HRI",
        executable="sm",
        name="hri",
        parameters=[
            os.path.join(get_package_share_directory("HRI"), "config", "lab.yaml")
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            load_motions,
            yolo_service,
            reid_service,
            vision_clip,
            eye_tracker,
            state_machine,
        ]
    )
