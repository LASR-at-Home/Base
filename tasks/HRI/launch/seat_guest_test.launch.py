import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    load_motions = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("skills"),
                "launch",
                "load_motions.launch.py",
            )
        )
    )

    yolo_service = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lasr_vision_yolo"),
                "launch",
                "service_launch.xml",
            )
        )
    )

    reid_service = Node(
        package="lasr_vision_reid",
        executable="service",
        name="lasr_vision_reid",
        output="screen",
    )

    seat_guest = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="HRI",
                executable="seat_guest",
                name="hri",
                parameters=[
                    os.path.join(
                        get_package_share_directory("HRI"), "config", "lab.yaml"
                    )
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([load_motions, yolo_service, reid_service, seat_guest])
