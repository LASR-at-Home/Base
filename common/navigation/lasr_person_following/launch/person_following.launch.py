"""person_tracker + follow_manager. Needs the YOLO service (/yolo/detect3d) and Nav2."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("follow_distance", default_value="1.0",
                              description="m to keep behind the host"),
        DeclareLaunchArgument("camera_latency", default_value="0.0",
                              description="s between image capture and its stamp"),
        DeclareLaunchArgument("head_tracking", default_value="true",
                              description="point the head at the host"),
        DeclareLaunchArgument("follow_max_speed", default_value="0.7",
                              description="m/s, Nav2 speed limit while following"),
        Node(package="lasr_person_following", executable="person_tracker",
             name="person_tracker", output="screen",
             parameters=[{"use_sim_time": use_sim_time,
                          "camera_latency": ParameterValue(
                              LaunchConfiguration("camera_latency"), value_type=float),
                          "head_tracking": ParameterValue(
                              LaunchConfiguration("head_tracking"), value_type=bool)}]),
        Node(package="lasr_person_following", executable="follow_manager",
             name="follow_manager", output="screen",
             parameters=[{"use_sim_time": use_sim_time,
                          "follow_distance": ParameterValue(LaunchConfiguration("follow_distance"),
                                                            value_type=float),
                          "follow_max_speed": ParameterValue(
                              LaunchConfiguration("follow_max_speed"), value_type=float)}]),
    ])
