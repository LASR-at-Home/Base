import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory("simulation")

    # map:=<name> loads maps/<name>.yaml (which points at maps/<name>.pgm)
    map_arg = DeclareLaunchArgument(
        "map",
        default_value="robocup_map",
        description="Name of the map in maps/ without extension (robocup_map or map)",
    )
    map_yaml = PathJoinSubstitution(
        [pkg_sim, "maps", [LaunchConfiguration("map"), ".yaml"]]
    )

    nav_params = os.path.join(pkg_sim, "config", "nav2_params_scan_raw.yaml")
    rviz_config = os.path.join(pkg_sim, "config", "mapping.rviz")

    # Start pose in the map frame. The map was built in the Gazebo world frame, so
    # this is also where TIAGo is teleported to (tiago_gazebo spawns it at 0, 0).
    INITIAL_POSE_X = "2.37254"
    INITIAL_POSE_Y = "-0.9725"
    INITIAL_POSE_YAW = "-0.1406"

    # TIAGo in the house world with MoveIt and grasping (rviz:=True also
    # opens MoveIt's RViz next to the mapping one below)
    tiago = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_sim, "launch", "sim.launch.py"))
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_yaml},
            {"use_sim_time": True},
        ],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[nav_params],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["map_server", "amcl"]},
        ],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav_params, {"use_sim_time": True}],
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[nav_params, {"use_sim_time": True}],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav_params, {"use_sim_time": True}],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav_params, {"use_sim_time": True}],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav_params, {"use_sim_time": True}],
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[nav_params, {"use_sim_time": True}],
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[nav_params, {"use_sim_time": True}],
        remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
    )

    navigation_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {
                "node_names": [
                    "controller_server",
                    "smoother_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower",
                    "velocity_smoother",
                ]
            },
        ],
    )

    nav2_nodes = TimerAction(
        period=15.0,
        actions=[
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
        ],
    )

    rviz = TimerAction(
        period=20.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        ],
    )

    qz = round(math.sin(float(INITIAL_POSE_YAW) / 2), 6)
    qw = round(math.cos(float(INITIAL_POSE_YAW) / 2), 6)
    teleport_msg = (
        '{state: {name: tiago, reference_frame: world, pose: {'
        f"position: {{x: {INITIAL_POSE_X}, y: {INITIAL_POSE_Y}, z: 0.01}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}}}}}}}"
    )
    initial_pose_msg = (
        '{"header": {"frame_id": "map"}, "pose": {"pose": {'
        f'"position": {{"x": {INITIAL_POSE_X}, "y": {INITIAL_POSE_Y}, "z": 0.0}}, '
        f'"orientation": {{"x": 0.0, "y": 0.0, "z": {qz}, "w": {qw}}}'
        '}, "covariance": [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}'
    )
    # Retry until TIAGo exists in Gazebo, then give AMCL the same pose
    place_robot = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "until ros2 service call /gazebo/set_entity_state "
            f"gazebo_msgs/srv/SetEntityState '{teleport_msg}' | grep -q 'success=True'; "
            "do sleep 2; done; sleep 15; "
            "ros2 topic pub -r 1 -t 5 /initialpose "
            f"geometry_msgs/msg/PoseWithCovarianceStamped '{initial_pose_msg}'",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            map_arg,
            tiago,
            nav2_nodes,
            rviz,
            place_robot,
        ]
    )
