"""person_tracker + follow_manager. Needs the YOLO service (/yolo/detect3d) and Nav2."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="lasr_person_following", executable="person_tracker",
             name="person_tracker", output="screen",
             parameters=[{"use_sim_time": use_sim_time}]),
        Node(package="lasr_person_following", executable="follow_manager",
             name="follow_manager", output="screen",
             parameters=[{"use_sim_time": use_sim_time}]),
    ])
