#!/usr/bin/env python3
"""
Test script for the Introduce state machine.

Scenario:
- guest1 (John) is already seated at position A
- guest2 (Sophie) just arrived and is seated at position B
- Robot looks at guest1 and introduces guest2 to them
- Robot looks at guest2 and introduces guest1 to them

Usage:
    ros2 run HRI introduce_test

Services required:
    - ros2 run lasr_vision_yolo yolo_service_node --ros-args -p use_sim_time:=true
    - ros2 run lasr_vision_reid service
    - ros2 run tts_engine tts_engine (+ configure + activate)
"""

import rclpy
import yasmin
import yasmin_ros
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header

from HRI.states.introduce import Introduce


def main():
    rclpy.init()
    yasmin_ros.set_ros_loggers()

    # Introduce guest2 to the already seated guest1
    sm = Introduce(guest_to_introduce="guest2", can_detect_second_guest=False)

    bb = yasmin.Blackboard()

    # Guest data — populated by GREET state in full task
    bb["guest_data"] = {
        "guest1": {
            "name": "John",
            "drink": "water",
            "detection": False,
            "seating_detection": False,
        },
        "guest2": {
            "name": "Sophie",
            "drink": "cola",
            "detection": False,
            "seating_detection": False,
        },
    }

    header = Header()
    header.frame_id = "map"

    # Where guest2 is sitting — robot looks here for second introduction
    # Replace with real coordinates from RViz
    bb["guest_seat_point"] = PointStamped(
        header=header,
        point=Point(x=-0.532, y=0.497, z=0.00247),
    )

    # Where guest1 is already seated — robot looks here to recognise and introduce guest2
    # Replace with real coordinates from RViz
    bb["seated_guest_locs"] = [
        Point(x=-0.219, y=0.0219, z=0.00247),
    ]

    yasmin.YASMIN_LOG_INFO("Starting Introduce test...")
    yasmin.YASMIN_LOG_INFO("Expected flow:")
    yasmin.YASMIN_LOG_INFO("  1. Robot looks at guest1 (John)")
    yasmin.YASMIN_LOG_INFO(
        "  2. Says: 'Hello John, this is Sophie. Their favourite drink is cola.'"
    )
    yasmin.YASMIN_LOG_INFO("  3. Robot looks back at guest2 (Sophie)")
    yasmin.YASMIN_LOG_INFO(
        "  4. Says: 'Hello Sophie, this is John. Their favourite drink is water.'"
    )

    try:
        outcome = sm(bb)
        yasmin.YASMIN_LOG_INFO(f"Introduce finished with outcome: {outcome}")
    except Exception as e:
        yasmin.YASMIN_LOG_ERROR(f"Introduce failed with exception: {e}")

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
