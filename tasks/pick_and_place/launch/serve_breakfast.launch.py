import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("pick_and_place"), "config", "config.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="pick_and_place",
                executable="test_serve_breakfast",
                name="pick_and_place",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="pick_and_place",
                executable="point_head_stub",
                name="point_head_stub",
                output="screen",
            ),
        ]
    )
