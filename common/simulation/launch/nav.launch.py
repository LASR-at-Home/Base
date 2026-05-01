import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_sim = get_package_share_directory('simulation')
    pkg_tiago_gazebo = get_package_share_directory('tiago_gazebo')

    my_models_path = os.path.join(pkg_sim, 'models')
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    os.environ['GAZEBO_MODEL_PATH'] = (existing + ':' + my_models_path) if existing else my_models_path

    map_yaml = os.path.join(pkg_sim, 'maps', 'map.yaml')
    nav_params = os.path.join(pkg_sim, 'config', 'nav2_params_scan_raw.yaml')

    tiago = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tiago_gazebo, 'launch', 'tiago_gazebo.launch.py')
        ),
        launch_arguments={
            'is_public_sim': 'True',
            'world_name': 'house',
            'moveit': 'True',
            'rviz': 'False',
            'navigation': 'False',
            'tuck_arm': 'True',
        }.items(),
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml},
            {'use_sim_time': True},
        ]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav_params]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']},
        ]
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('nav2_bringup').find('nav2_bringup'),
                'launch', 'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'map': map_yaml,
            'params_file': nav_params,
        }.items()
    )

    rviz = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        tiago,
        map_server,
        amcl,
        lifecycle_manager,
        navigation,
        rviz,
    ])
