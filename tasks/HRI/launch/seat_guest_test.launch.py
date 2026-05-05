import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    lab_yaml = os.path.join(
        get_package_share_directory("HRI"), "config", "lab.yaml"
    )
    motions_yaml = os.path.join(
        get_package_share_directory("skills"), "config", "motions.yaml"
    )
    motion_planner_yaml = os.path.join(
        get_package_share_directory("skills"), "config", "motion_planner.yaml"
    )

    play_motion2_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            get_package_share_directory("play_motion2"),
            "/launch/play_motion2.launch.py",
        ]),
        launch_arguments={
            "motions_file": motions_yaml,
            "motion_planner_config": motion_planner_yaml,
        }.items(),
    )

    return LaunchDescription([
        play_motion2_launch,
        Node(
            package="HRI",
            executable="seat_guest",
            name="hri",
            parameters=[lab_yaml],
            output="screen",
        ),
    ])