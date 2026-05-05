import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


BASE = os.path.expanduser("~/Desktop/Projects/lasr/Base")
WHISPER_PYTHONPATH = (
    f"{BASE}/build/lasr_speech_recognition_whisper:"
    f"{BASE}/build/lasr_speech_recognition_whisper/src"
)

# Initial pose of the robot in the map (x, y, yaw in radians)
INITIAL_POSE_X = "9.151"
INITIAL_POSE_Y = "-6.340"
INITIAL_POSE_YAW = "2.204"


def generate_launch_description():
    pkg_sim = get_package_share_directory("simulation")

    whisper_env = os.environ.copy()
    whisper_env["PYTHONPATH"] = WHISPER_PYTHONPATH + ":" + whisper_env.get("PYTHONPATH", "")

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, "launch", "nav.launch.py")
        )
    )

    rviz = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", os.path.join(pkg_sim, "config", "mapping.rviz")],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        ],
    )

    # deactivate → cleanup → configure → activate to force map publish
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

    # publish initial pose so AMCL localizes correctly from the start
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
                        f'"orientation": {{"x": 0.0, "y": 0.0, "z": {round(__import__("math").sin(float(INITIAL_POSE_YAW)/2), 6)}, "w": {round(__import__("math").cos(float(INITIAL_POSE_YAW)/2), 6)}}}'
                        '}, "covariance": [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}'
                    ),
                ],
                output="screen",
            )
        ],
    )

    whisper_server = ExecuteProcess(
        cmd=[
            "ros2", "run", "lasr_speech_recognition_whisper", "transcribe_microphone_server",
            "--no_warmup",
            "--energy_threshold", "6500",
            "--device", "cpu",
        ],
        env=whisper_env,
        output="screen",
    )

    state_machine = TimerAction(
        period=30.0,
        actions=[
            Node(
                package="autonomous_behaviour",
                executable="state_machine",
                name="autonomous_behaviour",
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        navigation,
        rviz,
        map_server_deactivate,
        map_server_cleanup,
        map_server_configure,
        map_server_activate,
        initial_pose,
        whisper_server,
        state_machine,
    ])
