import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    One-shot launch for the Pick and Place task.

    Brings up everything that used to be started in separate terminals EXCEPT
    the robot platform itself:
        - open-vocabulary detection service  (open_vocab/detect)  ← required
        - detection visualiser               (comes with the above)
        - LLM category-fallback service      (optional, use_llm:=true)
        - the task state machine             (state_machine)
        - point-head stub                    (/head_controller/point_head_action)

    Still launch SEPARATELY (the platform, unchanged between runs):
        - the simulator / robot bringup (camera, TF, controllers)
        - nav2 + localisation (map, /amcl_pose) — GoToLocation needs this

    Start the task after open_vocab has finished loading its model:
        ros2 topic pub --once /pick_and_place/start std_msgs/msg/Empty {}
    """

    pkg_pp = get_package_share_directory("pick_and_place")
    config = os.path.join(pkg_pp, "config", "config.yaml")

    open_vocab_launch = os.path.join(
        get_package_share_directory("lasr_vision_open_vocabulary"),
        "launch",
        "open_vocab.launch.py",
    )

    use_llm = LaunchConfiguration("use_llm")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_llm",
            default_value="false",
            description="Also start the storing_groceries LLM service "
                        "(category fallback). Forced onto CPU to avoid GPU OOM. "
                        "Most groceries resolve via CATEGORY_MAP, so default off.",
        ),

        # ── Perception: open-vocabulary detection (open_vocab/detect) ─────────
        # Reuses lasr_vision_open_vocabulary's own params.yaml (model / device /
        # weights). Keep your local fix there: grounding_dino_weights: '' and
        # model_device set for your GPU.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(open_vocab_launch),
        ),

        # ── Optional: LLM category-fallback service (CPU-forced) ─────────────
        Node(
            condition=IfCondition(use_llm),
            package="lasr_llm",
            executable="storing_groceries_service",
            name="storing_groceries_query_llm_service",
            output="screen",
            additional_env={"CUDA_VISIBLE_DEVICES": ""},
        ),

        # ── Task: state machine ──────────────────────────────────────────────
        Node(
            package="pick_and_place",
            executable="state_machine",
            name="pick_and_place",
            output="screen",
            parameters=[config],
        ),

        # ── Head stub: serves /head_controller/point_head_action ─────────────
        Node(
            package="pick_and_place",
            executable="point_head_stub",
            name="point_head_stub",
            output="screen",
        ),
    ])