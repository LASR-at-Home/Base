import os 

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node 

def generate_launch_description():
    config = os.path.join(get_package_share_directory('test_sm'),
    'config',
    'sim.yaml')
    

    return LaunchDescription([
        Node(
            package='test_sm',
            executable='main_sm',
            namespace='main_sm1',
            name='main_sm',
            parameters=[config]
        )
    ])



