"""Sim test: person_following + a simple driver that locks the host and follows."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("lasr_person_following")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "person_following.launch.py")),
            launch_arguments={"use_sim_time": "true"}.items()),
        Node(package="lasr_person_following", executable="follow_driver",
             name="follow_driver", output="screen", parameters=[{"use_sim_time": True}]),
    ])
