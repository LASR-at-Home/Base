import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    open_vocab_params = os.path.join(
        get_package_share_directory('lasr_vision_open_vocabulary'),
        'config', 'params.yaml'
    )
    manipulation_params = os.path.join(
        get_package_share_directory('lasr_manipulation_actions'),
        'config', 'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='lasr_vision_open_vocabulary',
            executable='open_vocabulary_node',
            name='lasr_vision_open_vocabulary',
            output='screen',
            parameters=[open_vocab_params],
        ),
        Node(
            package='lasr_vision_open_vocabulary',
            executable='detection_visualizer',
            name='detection_visualizer',
            output='screen',
        ),
        Node(
            package='lasr_manipulation_actions',
            executable='manipulation_actions_node',
            name='lasr_manipulation_actions',
            output='screen',
            parameters=[manipulation_params],
        ),
    ])
