import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ros_ip_arg = DeclareLaunchArgument(
        "ros_ip",
        default_value="10.68.0.139",
        description="IP address rosbridge will bind to (also passed to the Next.js UI as ROS_IP)",
    )

    port_arg = DeclareLaunchArgument(
        "port",
        default_value="9090",
        description="Port for the rosbridge WebSocket server",
    )

    ros_ip = LaunchConfiguration("ros_ip")
    port = LaunchConfiguration("port")

    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        parameters=[
            {
                "port": port,
                "address": ros_ip,
            }
        ],
    )

    ui_dir = os.path.join(
        get_package_share_directory("robot_ui"),
        "robot-interface",
    )

    next_server = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-c",
                    "npm install && chmod +x node_modules/.bin/next && node_modules/.bin/next build && node_modules/.bin/next start",
                ],
                cwd=ui_dir,
                additional_env={"ROS_IP": ros_ip},
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            ros_ip_arg,
            port_arg,
            rosbridge,
            next_server,
        ]
    )
