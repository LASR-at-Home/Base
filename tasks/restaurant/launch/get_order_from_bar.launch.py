import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    get_order_from_bar = Node(
        package="restaurant",
        executable="get_order_from_bar",
        name="restaurant",
        parameters=[
            os.path.join(
                get_package_share_directory("restaurant"), "config", "lab.yaml"
            )
        ],
        output="screen",
    )

    return LaunchDescription([get_order_from_bar])
