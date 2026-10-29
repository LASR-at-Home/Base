import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def append_env_path(name, path):
    # Set now so pal_gazebo.launch.py picks it up via environ
    existing = os.environ.get(name, "")
    os.environ[name] = (existing + ":" + path) if existing else path


def include(package, launch_file, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", launch_file)
        ),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    pkg_my_sim = get_package_share_directory("simulation")

    # Let Gazebo find our models and the world's grasp_assist plugin
    append_env_path("GAZEBO_MODEL_PATH", os.path.join(pkg_my_sim, "models"))
    append_env_path(
        "GAZEBO_PLUGIN_PATH", os.path.join(get_package_prefix("simulation"), "lib")
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="False",
        description="Show RViz with MoveIt's MotionPlanning display",
    )

    tiago_launch = include(
        "tiago_gazebo",
        "tiago_gazebo.launch.py",
        {
            "navigation": "False",
            "is_public_sim": "True",
            "world_name": "robocup",
            "moveit": "False",  # started below instead
            "tuck_arm": "True",
        },
    )

    # PAL's move_group. Executing MoveIt Task Constructor solutions (e.g.
    # pick and place) needs its ExecuteTaskSolutionCapability, which PAL's
    # launch file has no argument for, so it is added as a global parameter.
    # tiago_gazebo's own move_group can't be used for this: its include drops
    # global parameters.
    move_group = GroupAction(
        [
            SetParameter("capabilities", "move_group/ExecuteTaskSolutionCapability"),
            include("tiago_moveit_config", "move_group.launch.py", {"use_sim_time": "True"}),
        ],
        scoped=True,
    )

    # PAL's MoveIt RViz, which doesn't set use_sim_time itself
    rviz = GroupAction(
        [
            SetParameter("use_sim_time", True),
            include("tiago_moveit_config", "moveit_rviz.launch.py"),
        ],
        scoped=True,
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            rviz_arg,
            tiago_launch,
            move_group,
            rviz,
        ]
    )
