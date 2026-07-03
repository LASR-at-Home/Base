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

    open_vocab = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lasr_vision_open_vocabulary"),
                "launch",
                "open_vocab.launch.py",
            )
        )
    )

    state_machine = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="doing_laundry",
                executable="state_machine",
                name="doing_laundry",
                parameters=[
                    os.path.join(
                        get_package_share_directory("doing_laundry"),
                        "config",
                        "config.yaml",
                    )
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([load_motions, yolo_service, open_vocab, state_machine])