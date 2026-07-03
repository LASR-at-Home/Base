import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    state_machine = Node(
        package="HRI",
        executable="sm",
        name="hri",
        parameters=[
            os.path.join(get_package_share_directory("HRI"), "config", "lab_arena3.yaml")
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            state_machine
        ]
    )
