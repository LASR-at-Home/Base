import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_tablet_arg = DeclareLaunchArgument(
        "use_tablet",
        default_value="false",
        description="Use the robot_ui tablet for order taking instead of speech",
    )
    use_tablet = LaunchConfiguration("use_tablet")
    load_motions = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("skills"),
                "launch",
                "load_motions.launch.py",
            )
        )
    )

    bodypix_service = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lasr_vision_bodypix"),
                "launch",
                "bodypix_launch.py",
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
    llm_service = Node(
        package="lasr_llm",
        executable="restaurant_service",
        name="restaurant_llm",
        output="screen",
        additional_env={"CUDA_VISIBLE_DEVICES": ""},  # ← Qwen на CPU, звільняє 3ГБ
    )

    speech_recognition = Node(
        package="lasr_speech_recognition_whisper",
        executable="transcribe_microphone_server",
        name="transcribe_speech",
        output="screen",
        additional_env={"CUDA_VISIBLE_DEVICES": ""},
    )

    robot_ui = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_ui"),
                "launch",
                "robot_ui.launch.py",
            )
        )
    )

    restaurant = Node(
        package="restaurant",
        executable="sm",
        name="restaurant",
        parameters=[
            os.path.join(
                get_package_share_directory("restaurant"), "config", "lab.yaml"
            ),
            {"use_tablet": use_tablet},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            use_tablet_arg,
            load_motions,
            yolo_service,
            restaurant,
            llm_service,
            speech_recognition,
            robot_ui,
        ]
    )
