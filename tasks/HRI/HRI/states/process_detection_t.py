"""
#TODO: DELETE AFTER TESTING (AI GENERATED)
Standalone test for ProcessDetections.
Runs the state in isolation with fake Detection3D data — no robot, no full SM.

Usage:
    python3 test_process_detections.py

Scenarios (edit TEST_SCENARIO at the bottom):
    "empty_sofa"        - sofa empty, no chairs -> seats on sofa
    "one_on_sofa_left"  - one person on left of sofa -> seats on right
    "one_on_sofa_right" - one person on right of sofa -> seats on left
    "full_sofa"         - sofa full (2 people) -> falls back to chair
    "full_sofa_no_chair"- sofa full, no chair found -> fallback message
    "chair_occupied"    - sofa full, chair has person on it -> fallback message
"""

import rclpy
import smach
import threading
import numpy as np

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from geometry_msgs.msg import Point
from shapely.geometry import Polygon as ShapelyPolygon

# Import directly from your package — adjust the path if running outside colcon
from lasr_vision_interfaces.msg import Detection3D
from .seat_guest import ProcessDetections  # adjust if module path differs


# ── Helpers ──────────────────────────────────────────────────────────────────


def make_detection(
    name: str, x: float, y: float, z: float, xywh: list = None
) -> Detection3D:
    """Build a minimal Detection3D message."""
    d = Detection3D()
    d.name = name
    d.point = Point(x=x, y=y, z=z)
    d.xywh = xywh if xywh is not None else [0, 0, 10, 10]
    return d


def run_test(node: Node, scenario: str, scenarios_data: dict, geometry_data: dict):
    """Runs a single test case for a given scenario."""

    if scenario not in scenarios_data:
        print(
            f"Unknown scenario '{scenario}'. Choose from: {list(scenarios_data.keys())}"
        )
        return

    data = scenarios_data[scenario]

    # ── Build and run the state ───────────────────────────────────────────────
    state = ProcessDetections(
        node=node,
        sofa_point=geometry_data["sofa_point"],
        left_sofa_area=geometry_data["left_sofa_polygon"],
        right_sofa_area=geometry_data["right_sofa_polygon"],
        max_people_on_sofa=2,
    )

    # Populate userdata manually
    ud = smach.UserData()
    ud.sofa_detections = data["sofa_detections"]
    ud.non_sofa_detections = data["non_sofa_detections"]

    print(f"\n{'='*60}")
    print(f"  Scenario: {scenario}")
    print(f"  sofa_detections:     {len(ud.sofa_detections)} detection(s)")
    print(f"  non_sofa_detections: {len(ud.non_sofa_detections)} detection(s)")
    print(f"{'-'*60}")

    outcome = state.execute(ud)

    print(f"\n  Outcome:        {outcome}")
    print(f"  seating_string: {getattr(ud, 'seating_string', '<not set>')}")
    print(f"{'='*60}\n")


# ── Entry point ───────────────────────────────────────────────────────────────


def main():
    # ── Geometry (matches main() in seat_guest.py) ────────────────────────────
    sofa_area = {
        "top_left": np.array([3.0941781997680664, 1.1541430950164795]),
        "top_right": np.array([3.091914653778076, -0.9371256828308105]),
        "bottom_right": np.array([2.128192901611328, -0.9390065670013428]),
        "bottom_left": np.array([2.3177337646484375, 0.8923218250274658]),
    }
    sofa_middle_top = (sofa_area["top_right"] + sofa_area["top_left"]) / 2
    sofa_middle_bottom = (sofa_area["bottom_left"] + sofa_area["bottom_right"]) / 2

    geometry_data = {
        "left_sofa_polygon": ShapelyPolygon(
            [
                sofa_area["top_left"],
                sofa_middle_top,
                sofa_middle_bottom,
                sofa_area["bottom_left"],
            ]
        ),
        "right_sofa_polygon": ShapelyPolygon(
            [
                sofa_middle_top,
                sofa_area["top_right"],
                sofa_area["bottom_right"],
                sofa_middle_bottom,
            ]
        ),
        "sofa_point": Point(
            x=2.3416929244995117, y=0.07656313478946686, z=-0.0012598037719726562
        ),
    }

    # ── Fake detections per scenario ──────────────────────────────────────────
    scenarios_data = {
        "empty_sofa": {
            "sofa_detections": [],
            "non_sofa_detections": [],
        },
        "one_on_sofa_left": {
            # Person sitting in left polygon (high x, high y)
            "sofa_detections": [make_detection("person", 2.8, 0.9, 0.5)],
            "non_sofa_detections": [],
        },
        "one_on_sofa_right": {
            # Person sitting in right polygon (high x, negative y)
            "sofa_detections": [make_detection("person", 2.8, -0.7, 0.5)],
            "non_sofa_detections": [],
        },
        "full_sofa": {
            "sofa_detections": [
                make_detection("person", 2.8, 0.9, 0.5),
                make_detection("person", 2.8, -0.7, 0.5),
            ],
            # One empty chair in non-sofa area
            "non_sofa_detections": [
                make_detection("chair", 1.5, 0.0, 0.0, xywh=[100, 100, 80, 80]),
            ],
        },
        "full_sofa_no_chair": {
            "sofa_detections": [
                make_detection("person", 2.8, 0.9, 0.5),
                make_detection("person", 2.8, -0.7, 0.5),
            ],
            "non_sofa_detections": [],
        },
        "chair_occupied": {
            "sofa_detections": [
                make_detection("person", 2.8, 0.9, 0.5),
                make_detection("person", 2.8, -0.7, 0.5),
            ],
            # Chair with a person heavily overlapping it (>50%)
            "non_sofa_detections": [
                make_detection("chair", 1.5, 0.0, 0.0, xywh=[100, 100, 80, 80]),
                make_detection("person", 1.5, 0.0, 0.5, xywh=[110, 110, 60, 60]),
            ],
        },
    }

    # ── ROS2 setup ────────────────────────────────────────────────────────────
    rclpy.init()
    node = rclpy.create_node("hri_test")

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # --- Run all scenarios ---
    for scenario_name in scenarios_data.keys():
        run_test(node, scenario_name, scenarios_data, geometry_data)

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
