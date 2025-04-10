from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value=TextSubstitution(text="/head_front_camera/rgb/image_raw"),
        description="Input image topic for keypoint relay",
    )

    # Path to the BodyPix launch file
    yolo_launch_file = os.path.join(
        get_package_share_directory("lasr_vision_yolov8"),
        "launch",
        "service_launch.py",
    )

    # Create the Cropped Detection service node
    cropped_detection_node = Node(
        package="lasr_vision_cropped_detection",
        executable="cropped_detection_services.py",
        name="cropped_detection_services",
        output="screen",
        # parameters=[{"preload": LaunchConfiguration("preload")}],
    )

    # Return the launch description
    return LaunchDescription(
        [
            # preload_arg,  # Argument declaration
            image_topic_arg,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(yolo_launch_file),
            ),
            cropped_detection_node,  # Node for the Cropped Detection service
        ]
    )


# <launch>
#     <node name="yolo" pkg="lasr_vision_yolov8" type="service" output="screen"/>
#     <node name="cropped_detection" pkg="lasr_vision_cropped_detection" type="service.py" output="screen"/>
# </launch>
