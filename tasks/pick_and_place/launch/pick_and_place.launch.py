import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for the Pick and Place task using YOLO detection.

    Launch SEPARATELY before running this:
        - Simulator / robot bringup
        - Nav2 + localisation

    Start the task after everything is ready:
        ros2 topic pub --once /pick_and_place/start std_msgs/msg/Empty {}
    """
    pkg_pp = get_package_share_directory("pick_and_place")
    config = os.path.join(pkg_pp, "config", "config.yaml")

    use_sim = LaunchConfiguration("use_sim")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Set to false when running on the real robot "
                        "to disable the point_head_stub.",
        ),

        # ── Perception: YOLO detection ────────────────────────────────────────
        Node(
            package="lasr_vision_yolo",
            executable="yolo_service_node",
            name="lasr_vision_yolo",
            output="screen",
            parameters=[{
        "preload": ["/path/to/lasr_vision_yolo/models/best.pt"]}],
        ),

        # ── Task: state machine ───────────────────────────────────────────────
        Node(
            package="pick_and_place",
            executable="state_machine",
            name="pick_and_place",
            output="screen",
            parameters=[config],
        ),

        # ── Head stub (simulation only) ───────────────────────────────────────
        # Remove this when testing on the real robot by passing use_sim:=false
        Node(
            condition=IfCondition(False),
            package="pick_and_place",
            executable="point_head_stub",
            name="point_head_stub",
            output="screen",
        ),
    ])