import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    open_vocab_params = os.path.join(
        get_package_share_directory('open_vocabulary_models'),
        'config', 'params.yaml'
    )
    manipulation_params = os.path.join(
        get_package_share_directory('manipulation_actions'),
        'config', 'params.yaml'
    )

    return LaunchDescription([
        # Vision core: loads Grounding DINO, waits for service calls
        Node(
            package='open_vocabulary_models',
            executable='open_vocabulary_node',
            name='open_vocabulary_models',
            output='screen',
            parameters=[open_vocab_params],
        ),
        # Detection node: listens /detect, publishes /object_centroids + RViz markers
        Node(
            package='open_vocabulary_models',
            executable='detection_visualizer',
            name='detection_visualizer',
            output='screen',
        ),
        # Manipulation node: listens /command
        Node(
            package='manipulation_actions',
            executable='manipulation_actions_node',
            name='manipulation_actions',
            output='screen',
            parameters=[manipulation_params],
        ),
    ])
