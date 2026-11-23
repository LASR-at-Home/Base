"""person_tracker + follow_manager. Needs the YOLO service (/yolo/detect3d) and Nav2."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("global_frame", default_value="odom",
                              description="follow_manager frame: map or odom"),
        DeclareLaunchArgument("reid_relaxed", default_value="true",
                              description="relaxed re-identification of a lost host"),
        Node(package="lasr_person_following", executable="person_tracker",
             name="person_tracker", output="screen",
             parameters=[{"use_sim_time": use_sim_time,
                          "reid_relaxed": ParameterValue(LaunchConfiguration("reid_relaxed"), value_type=bool)}]),
        Node(package="lasr_person_following", executable="follow_manager",
             name="follow_manager", output="screen",
             parameters=[{"use_sim_time": use_sim_time,
                          "global_frame": LaunchConfiguration("global_frame")}]),
    ])
