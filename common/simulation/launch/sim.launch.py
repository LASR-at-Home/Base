import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_my_sim = get_package_share_directory('simulation')
    pkg_tiago_gazebo = get_package_share_directory('tiago_gazebo')

    my_models_path = os.path.join(pkg_my_sim, 'models')

    # Inject our models path now so pal_gazebo.launch.py picks it up via environ
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    os.environ['GAZEBO_MODEL_PATH'] = (existing + ':' + my_models_path) if existing else my_models_path

    tiago_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tiago_gazebo, 'launch', 'tiago_gazebo.launch.py')
        ),
        launch_arguments={
            'navigation': 'False',
            'is_public_sim': 'True',
            'world_name': 'house',
            'moveit': 'True',
            'rviz': 'False',
            'tuck_arm': 'True',
        }.items(),
    )

    return LaunchDescription([
        tiago_launch,
    ])
