import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('lasr_vision_open_vocabulary'),
        'config', 'params.yaml'
    )
    return LaunchDescription([
        Node(
            package='lasr_vision_open_vocabulary',
            executable='open_vocabulary_node',
            name='lasr_vision_open_vocabulary',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='lasr_vision_open_vocabulary',
            executable='detection_visualizer',
            name='detection_visualizer',
            output='screen',
        ),
    ])
