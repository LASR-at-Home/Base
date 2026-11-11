import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory("simulation")

    maps_dir = os.path.join(pkg_sim, "maps")
    map_pgm = os.path.join(maps_dir, "map.pgm")
    map_yaml = "/tmp/map_resolved.yaml"
    with open(os.path.join(maps_dir, "map.yaml")) as f:
        content = f.read()
    with open(map_yaml, "w") as f:
        f.write(content.replace("image: map.pgm", f"image: {map_pgm}"))

    nav_params = os.path.join(pkg_sim, "config", "nav2_params_scan_raw.yaml")
    rviz_config = os.path.join(pkg_sim, "config", "mapping.rviz")

    INITIAL_POSE_X = "9.151"
    INITIAL_POSE_Y = "-6.340"
    INITIAL_POSE_YAW = "2.204"

    # The simulation (Gazebo, TIAGo, MoveIt) is launched first; everything below
    # for navigation only starts once it is up.
    sim = IncludeLaunchDescription(
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

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    initial_pose_msg = (
        '{"header": {"frame_id": "map"}, "pose": {"pose": {'
        f'"position": {{"x": {INITIAL_POSE_X}, "y": {INITIAL_POSE_Y}, "z": 0.0}}, '
        f'"orientation": {{"x": 0.0, "y": 0.0, "z": {round(math.sin(float(INITIAL_POSE_YAW)/2), 6)}, "w": {round(math.cos(float(INITIAL_POSE_YAW)/2), 6)}}}'
        '}, "covariance": [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}'
    )
    # AMCL only accepts the pose once it is active
    initial_pose = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "until ros2 lifecycle get /amcl 2>/dev/null | grep -q '^active'; do sleep 1; done; "
            "ros2 topic pub --once /initialpose "
            f"geometry_msgs/msg/PoseWithCovarianceStamped '{initial_pose_msg}'",
        ],
        output="screen",
    )

    # TIAGo's lidar publishes once the robot is spawned, so wait for that
    robot_ready = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "echo",
            "--once",
            "--qos-reliability",
            "best_effort",
            "/scan_raw",
        ],
        output="log",
    )
    nav = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_ready,
            on_exit=[
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
                initial_pose,
            ],
        )
    )

    return LaunchDescription(
        [
            sim,
            robot_ready,
            nav,
        ]
    )
