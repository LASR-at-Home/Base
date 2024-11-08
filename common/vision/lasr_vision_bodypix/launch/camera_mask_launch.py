from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        'model', default_value='resnet50', description='Model to use for the demo'
    )

    # Path to the BodyPix launch file
    bodypix_launch_file = os.path.join(
        get_package_share_directory('lasr_vision_bodypix'), 'launch', 'bodypix_launch.py'
    )

    # Path to the v4l2_camera launch file (replacement for video_stream_opencv)
    v4l2_camera_launch_file = os.path.join(
        get_package_share_directory('v4l2_camera'), 'launch', 'v4l2_camera_node.launch.py'
    )

    return LaunchDescription([
        # Declare the model argument
        model_arg,

        # Include BodyPix launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bodypix_launch_file),
            launch_arguments={
                'preload': LaunchConfiguration('model')
            }.items(),
        ),

        # Show debug topic using rqt_image_view
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='image_view',
            output='screen',
            arguments=[
                TextSubstitution(text='/bodypix/debug/'), 
                LaunchConfiguration('model')
            ],  # Constructs the topic path dynamically
        ),

        # Start the keypoint relay service
        Node(
            package='lasr_vision_bodypix',
            executable='keypoint_relay.py',  # Removed .py extension, assuming installed without it
            name='keypoint_relay',
            output='screen',
            arguments=[LaunchConfiguration('model')],
        ),

        # Include the v4l2_camera launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(v4l2_camera_launch_file),
            launch_arguments={'image_size': '640x480'}.items(),
        ),
    ])

