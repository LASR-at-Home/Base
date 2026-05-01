import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    lab_yaml = os.path.join(
        get_package_share_directory("HRI"), "config", "lab.yaml"
    )
    motions_yaml = os.path.join(
        get_package_share_directory("skills"), "config", "motions.yaml"
    )

    return LaunchDescription([
        Node(
            package="HRI",
            executable="seat_guest",
            name="hri",
            parameters=[lab_yaml, motions_yaml],
            output="screen",
        )
    ])