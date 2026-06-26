import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# CLIP recognition candidates: the open-vocab detector LOCALISES objects (boxes),
# then CLIP re-labels each crop against THIS list (fixes "Pringles -> cup"). Names
# must be CATEGORY_MAP-friendly so routing works (see classify_category.py).
CLIP_CANDIDATES = [
    "pringles", "iced tea", "apple", "milk", "can", "coke", "cup", "sprite","bowl", "spoon", "water bottle", "banana", "cereal"
]

def generate_launch_description():
    """
    One-shot launch for the Pick and Place task.

    Perception = open-vocab detection (localisation) + CLIP rerank (recognition),
    both inside the lasr_vision_open_vocabulary node (its venv already has
    transformers/torch — CLIP comes for free, no new deps).

    Still launch SEPARATELY: simulator / robot bringup + nav2 + localisation.

    Start after the model has loaded:
        ros2 topic pub --once /pick_and_place/start std_msgs/msg/Empty {}
    """
    pkg_pp = get_package_share_directory("pick_and_place")
    config = os.path.join(pkg_pp, "config", "config.yaml")

    ov_params = os.path.join(
        get_package_share_directory("lasr_vision_open_vocabulary"),
        "config",
        "params.yaml",
    )

    use_llm = LaunchConfiguration("use_llm")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_llm",
            default_value="false",
            description="Also start the storing_groceries LLM service "
                        "(category fallback). Forced onto CPU. Default off.",
        ),

        # ── Perception: open-vocab detection + CLIP recognition rerank ────────
        # Node(
        #     package="lasr_vision_open_vocabulary",
        #     executable="open_vocabulary_node",
        #     name="lasr_vision_open_vocabulary",
        #     output="screen",
        #     parameters=[
        #         ov_params,
        #         {"clip_rerank": True, "clip_candidates": CLIP_CANDIDATES},
        #     ],
        # ),
        Node(
            package="lasr_vision_open_vocabulary",
            executable="detection_visualizer",
            name="detection_visualizer",
            output="screen",
        ),

        # ── Optional: LLM category-fallback service (CPU-forced) ─────────────
        # Node(
        #     condition=IfCondition(use_llm),
        #     package="lasr_llm",
        #     executable="storing_groceries_service",
        #     name="storing_groceries_query_llm_service",
        #     output="screen",
        #     additional_env={"CUDA_VISIBLE_DEVICES": ""},
        # ),

        # ── Task: state machine ──────────────────────────────────────────────
        Node(
            package="pick_and_place",
            executable="state_machine",
            name="pick_and_place",
            output="screen",
            parameters=[config],
        ),

        # ── Head stub ────────────────────────────────────────────────────────
        Node(
            package="pick_and_place",
            executable="point_head_stub",
            name="point_head_stub",
            output="screen",
        ),
        ExecuteProcess(
            cmd=["bash", "-c",
                 "curl -sf localhost:11434/api/tags >/dev/null 2>&1 "
                 "&& echo 'ollama already running' "
                 "|| exec ollama serve"],
            name="ollama_serve",
            output="screen",
        ),
    ])