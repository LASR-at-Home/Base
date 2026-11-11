import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory("simulation")
    nav_params = os.path.join(pkg_sim, "config", "nav2_params_scan_raw.yaml")
    sim_time = {"use_sim_time": True}

    # Start pose in the map frame. The map was built in the Gazebo world frame, so
    # this is also where TIAGo is teleported to (tiago_gazebo spawns it at 0, 0).
    x, y, yaw = 2.37254, -0.9725, -0.1406
    qz, qw = round(math.sin(yaw / 2), 6), round(math.cos(yaw / 2), 6)

    # world:=<name> loads worlds/<name>.world and the matching maps/<name>.yaml
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="robocup",
        description="World and map name in worlds/ and maps/ without extension (robocup or house)",
    )
    world = LaunchConfiguration("world")

    # 1. The simulation: Gazebo, TIAGo and MoveIt
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_sim, "launch", "sim.launch.py")),
        launch_arguments={"world": world}.items(),
    )

    # 2. Once TIAGo is spawned (its lidar is publishing), start navigation
    sim_ready = ExecuteProcess(
        cmd="ros2 topic echo --once --qos-reliability best_effort /scan_raw".split(),
        output="log",
    )

    def nav2(package, name, remappings=None):
        return Node(
            package=package,
            executable=name,
            name=name,
            output="screen",
            parameters=[nav_params, sim_time],
            remappings=remappings,
        )

    def lifecycle_manager(name, node_names):
        return Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name=name,
            output="screen",
            parameters=[sim_time, {"autostart": True}, {"node_names": node_names}],
        )

    localisation = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                {"yaml_filename": PathJoinSubstitution([pkg_sim, "maps", [world, ".yaml"]])},
                sim_time,
            ],
        ),
        nav2("nav2_amcl", "amcl"),
        lifecycle_manager("lifecycle_manager_localization", ["map_server", "amcl"]),
    ]

    navigation = [
        nav2("nav2_controller", "controller_server", [("cmd_vel", "cmd_vel_nav")]),
        nav2("nav2_smoother", "smoother_server"),
        nav2("nav2_planner", "planner_server"),
        nav2("nav2_behaviors", "behavior_server"),
        nav2("nav2_bt_navigator", "bt_navigator"),
        nav2("nav2_waypoint_follower", "waypoint_follower"),
        nav2(
            "nav2_velocity_smoother",
            "velocity_smoother",
            [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
        ),
        lifecycle_manager(
            "lifecycle_manager_navigation",
            [
                "controller_server",
                "smoother_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
                "waypoint_follower",
                "velocity_smoother",
            ],
        ),
    ]

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(pkg_sim, "config", "mapping.rviz")],
        parameters=[sim_time],
        output="screen",
    )

    # Teleport TIAGo to its start pose (retrying until Gazebo's service answers),
    # then give AMCL the same pose once it is active
    teleport_msg = (
        f"{{state: {{name: tiago, reference_frame: world, pose: {{"
        f"position: {{x: {x}, y: {y}, z: 0.01}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}}}}}}}"
    )
    initial_pose_msg = (
        '{"header": {"frame_id": "map"}, "pose": {"pose": {'
        f'"position": {{"x": {x}, "y": {y}, "z": 0.0}}, '
        f'"orientation": {{"x": 0.0, "y": 0.0, "z": {qz}, "w": {qw}}}'
        '}, "covariance": [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}'
    )
    place_robot = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "until ros2 service call /gazebo/set_entity_state "
            f"gazebo_msgs/srv/SetEntityState '{teleport_msg}' | grep -q 'success=True'; "
            "do sleep 2; done; "
            "until ros2 lifecycle get /amcl 2>/dev/null | grep -q '^active'; do sleep 1; done; "
            "ros2 topic pub -r 1 -t 5 /initialpose "
            f"geometry_msgs/msg/PoseWithCovarianceStamped '{initial_pose_msg}'",
        ],
        output="screen",
    )

    nav = RegisterEventHandler(
        OnProcessExit(
            target_action=sim_ready,
            on_exit=[*localisation, *navigation, rviz, place_robot],
        )
    )

    return LaunchDescription([world_arg, sim, sim_ready, nav])
