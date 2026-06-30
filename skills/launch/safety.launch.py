import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    state_machine = Node(
        package='skills',
        executable='safety',
        name='safety',
        parameters=[
            os.path.join(get_package_share_directory('skills', 'config', 'safety.yaml'))
        ],
        output='screen'
    )
    
    return LaunchDescription(state_machine)
