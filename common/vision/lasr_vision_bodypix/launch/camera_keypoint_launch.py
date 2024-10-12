from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        'model', default_value='resnet50', description='Model to use for the demo'
    )

    # Path to the BodyPix launch file
    bodypix_launch_file = os.path.join(
        get_package_share_directory('lasr_vision_bodypix'), 'launch', 'bodypix.launch.py'
    )

    # Path to the video stream launch file
    video_stream_launch_file = os.path.join(
        get_package_share_directory('video_stream_opencv'), 'launch', 'camera.launch.py'
    )

    return LaunchDescription([
        # Declare the model argument
        model_arg,

        # Include BodyPix launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bodypix_launch_file),
            launch_arguments={'preload': LaunchConfiguration('model')}.items(),  # Corrected argument
        ),

        # Show debug topic using rqt_image_view
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='image_view',
            output='screen',
            arguments=[LaunchConfiguration('model')],  # Corrected arguments
        ),

        # Start the keypoint relay service (assuming the executable is installed without .py)
        Node(
            package='lasr_vision_bodypix',
            executable='keypoint_relay',  # Remove the .py extension
            name='keypoint_relay',
            output='screen',
            arguments=[LaunchConfiguration('model')],  # Corrected arguments
        ),

        # Include the video stream launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(video_stream_launch_file),
            launch_arguments={'visualize': 'true'}.items(),
        ),
    ])
