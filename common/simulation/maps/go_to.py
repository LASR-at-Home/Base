#!/usr/bin/env python3
"""Test GoToLocation skill from command line.

Usage:
    python3 go_to.py --location kitchen
"""

import argparse
import sys
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Point, Pose, Quaternion

sys.path.insert(0, str(Path(__file__).parents[3] / "skills" / "src"))
from lasr_skills import GoToLocation

MAP_DIR = Path(__file__).parent
LOCATIONS_YAML = MAP_DIR / "locations.yaml"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--location", required=False, help="Location name from locations.yaml")
    parser.add_argument("--list", action="store_true", help="List available locations")
    args = parser.parse_args()

    with open(LOCATIONS_YAML) as f:
        data = yaml.safe_load(f)
    locations = data.get("locations", {})

    if args.list or not args.location:
        print("Available locations:")
        for name in locations:
            loc = locations[name]
            print(f"  {name}: x={loc['position']['x']}, y={loc['position']['y']}")
        return

    if args.location not in locations:
        print(f"ERROR: '{args.location}' not found. Available: {list(locations.keys())}")
        sys.exit(1)

    loc = locations[args.location]
    pose = Pose(
        position=Point(
            x=float(loc["position"]["x"]),
            y=float(loc["position"]["y"]),
            z=float(loc["position"].get("z", 0.0)),
        ),
        orientation=Quaternion(
            x=float(loc["orientation"]["x"]),
            y=float(loc["orientation"]["y"]),
            z=float(loc["orientation"]["z"]),
            w=float(loc["orientation"]["w"]),
        ),
    )

    rclpy.init()
    node = rclpy.create_node("go_to_location_test")
    node.get_logger().info(f"Going to '{args.location}': x={pose.position.x}, y={pose.position.y}")

    state = GoToLocation(node=node, location=pose)
    outcome = state.execute(userdata={})
    node.get_logger().info(f"Outcome: {outcome}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
