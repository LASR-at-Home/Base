import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import math


def generate_launch_description():
    pkg_sim = get_package_share_directory('simulation')
    pkg_tiago_gazebo = get_package_share_directory('tiago_gazebo')

    my_models_path = os.path.join(pkg_sim, 'models')
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    os.environ['GAZEBO_MODEL_PATH'] = (existing + ':' + my_models_path) if existing else my_models_path

    map_yaml = os.path.join(pkg_sim, 'maps', 'map.yaml')
    nav_params = os.path.join(pkg_sim, 'config', 'nav2_params_scan_raw.yaml')

    # Initial pose of the robot in the map (x, y, yaw in radians)
    INITIAL_POSE_X = "9.151"
    INITIAL_POSE_Y = "-6.340"
    INITIAL_POSE_YAW = "2.204"

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

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav_params, {'use_sim_time': True}],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav_params, {'use_sim_time': True}],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav_params, {'use_sim_time': True}],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav_params, {'use_sim_time': True}],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav_params, {'use_sim_time': True}],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav_params, {'use_sim_time': True}],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav_params, {'use_sim_time': True}],
        remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
    )

    navigation_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ]},
        ],
    )

    rviz_config = os.path.join(pkg_sim, 'config', 'mapping.rviz')
    rviz = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
        ],
    )

    map_server_deactivate = TimerAction(
        period=20.0,
        actions=[ExecuteProcess(cmd=["ros2", "lifecycle", "set", "/map_server", "deactivate"], output="screen")]
    )
    map_server_cleanup = TimerAction(
        period=21.0,
        actions=[ExecuteProcess(cmd=["ros2", "lifecycle", "set", "/map_server", "cleanup"], output="screen")]
    )
    map_server_configure = TimerAction(
        period=22.0,
        actions=[ExecuteProcess(cmd=["ros2", "lifecycle", "set", "/map_server", "configure"], output="screen")]
    )
    map_server_activate = TimerAction(
        period=23.0,
        actions=[ExecuteProcess(cmd=["ros2", "lifecycle", "set", "/map_server", "activate"], output="screen")]
    )

    initial_pose = TimerAction(
        period=24.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "topic", "pub", "--once",
                    "/initialpose",
                    "geometry_msgs/msg/PoseWithCovarianceStamped",
                    (
                        '{"header": {"frame_id": "map"}, "pose": {"pose": {'
                        f'"position": {{"x": {INITIAL_POSE_X}, "y": {INITIAL_POSE_Y}, "z": 0.0}}, '
                        f'"orientation": {{"x": 0.0, "y": 0.0, "z": {round(math.sin(float(INITIAL_POSE_YAW)/2), 6)}, "w": {round(math.cos(float(INITIAL_POSE_YAW)/2), 6)}}}'
                        '}, "covariance": [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}'
                    ),
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        tiago,
        map_server,
        amcl,
        lifecycle_manager,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        navigation_lifecycle_manager,
        rviz,
        map_server_deactivate,
        map_server_cleanup,
        map_server_configure,
        map_server_activate,
        initial_pose,
    ])
