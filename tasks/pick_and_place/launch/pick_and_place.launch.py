import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_pp = get_package_share_directory("pick_and_place")
    config = os.path.join(pkg_pp, "config", "config.yaml")

    yolo_launch = os.path.join(
        get_package_share_directory("lasr_vision_yolo"),
        "launch",
        "service_launch.xml"
    )

    use_sim = LaunchConfiguration("use_sim")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Set to false when running on the real robot "
                        "to disable the point_head_stub.",
        ),

        # ── Perception: YOLO detection ────────────────────────────────────
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(yolo_launch),
        ),

        # ── Task: state machine ───────────────────────────────────────────
        Node(
            package="pick_and_place",
            executable="state_machine",
            name="pick_and_place",
            output="screen",
            parameters=[config],
        ),

        # ── Head stub (simulation only) ───────────────────────────────────
        Node(
            condition=IfCondition(use_sim),
            package="pick_and_place",
            executable="point_head_stub",
            name="point_head_stub",
            output="screen",
        ),
    ])