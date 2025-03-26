from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value=TextSubstitution(text="/head_front_camera/rgb/image_raw"),
        description="Input image topic for keypoint relay",
    )

    # Path to the BodyPix launch file
    bodypix_launch_file = os.path.join(
        get_package_share_directory("lasr_vision_bodypix"),
        "launch",
        "bodypix_launch.py",
    )

    return LaunchDescription(
        [
            image_topic_arg,

            # Include BodyPix launch file
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bodypix_launch_file),
            ),

            # Show debug topic using rqt_image_view
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                name="image_view",
                output="screen",
            ),

            # Start the keypoint relay service
            Node(
                package="lasr_vision_bodypix",
                executable="keypoint_relay.py",  # Specifying the subdirectory
                name="keypoint_relay",
                output="screen",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                    }
                ]
            ),
        ]
    )
