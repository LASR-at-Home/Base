import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    motions_yaml = os.path.join(
        get_package_share_directory("skills"), "config", "motions.yaml"
    )

    param_load = ExecuteProcess(
        cmd=["ros2", "param", "load", "/play_motion2_mgr", motions_yaml],
        output="screen",
    )
    deactivate  = ExecuteProcess(cmd=["ros2", "lifecycle", "set", "/play_motion2_mgr", "deactivate"],  output="screen")
    cleanup     = ExecuteProcess(cmd=["ros2", "lifecycle", "set", "/play_motion2_mgr", "cleanup"],     output="screen")
    configure   = ExecuteProcess(cmd=["ros2", "lifecycle", "set", "/play_motion2_mgr", "configure"],   output="screen")
    activate    = ExecuteProcess(cmd=["ros2", "lifecycle", "set", "/play_motion2_mgr", "activate"],    output="screen")

    return LaunchDescription([
        param_load,
        RegisterEventHandler(OnProcessExit(target_action=param_load, on_exit=[deactivate,
            RegisterEventHandler(OnProcessExit(target_action=deactivate, on_exit=[cleanup,
                RegisterEventHandler(OnProcessExit(target_action=cleanup, on_exit=[configure,
                    RegisterEventHandler(OnProcessExit(target_action=configure, on_exit=[activate]))
                ]))
            ]))
        ])),
    ])