from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

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
        # Declare model argument
        model_arg,

        # Include BodyPix launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bodypix_launch_file),
            launch_arguments={'preload': LaunchConfiguration('model')}.items(),  # Corrected LaunchConfiguration usage
        ),

        # Show debug topic using rqt_image_view
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='image_view',
            output='screen',
            # Corrected: Directly passing LaunchConfiguration in arguments
            arguments=[LaunchConfiguration('model')],
        ),

        # Start the mask relay service
        Node(
            package='lasr_vision_bodypix',
            executable='mask_relay',  # Remove .py extension, assuming it's installed as executable
            name='mask_relay',
            output='screen',
            # Corrected: Directly passing LaunchConfiguration in arguments
            arguments=['/camera/image_raw', LaunchConfiguration('model')],
        ),

        # Include video stream launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(video_stream_launch_file),
            launch_arguments={'visualize': 'true'}.items(),
        ),
    ])
