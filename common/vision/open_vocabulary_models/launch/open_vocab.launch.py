import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('open_vocabulary_models'),
        'config', 'params.yaml'
    )
    return LaunchDescription([
        Node(
            package='open_vocabulary_models',
            executable='open_vocabulary_node',
            name='open_vocabulary_models',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='open_vocabulary_models',
            executable='detection_visualizer',
            name='detection_visualizer',
            output='screen',
        ),
    ])
