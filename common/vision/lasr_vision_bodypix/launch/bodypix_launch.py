from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Create the BodyPix service node
    bodypix_node = Node(
        package="lasr_vision_bodypix",
        executable="bodypix_services.py",
        name="bodypix_services",
        output="screen",
        # parameters=[{"preload": LaunchConfiguration("preload")}],
    )

    # Return the launch description
    return LaunchDescription(
        [
            # preload_arg,  # Argument declaration
            bodypix_node,  # Node for the BodyPix service
        ]
    )
