#!/usr/bin/env python3
import rclpy
import yasmin
import yasmin_ros
from pick_and_place.states.serve_breakfast import ServeBreakfast


def main():

    rclpy.init()

    yasmin_ros.set_ros_loggers()

    bb = yasmin.Blackboard()

    bb["detected_objects"] = []

    bb["debug_images"] = []

    bb["selected_object"] = None

    bb["selected_object_name"] = ""

    bb["last_rgb_image"] = None

    bb["object_name"] = ""

    # Add these three for InstructPick/InstructPlace

    bb["object_category"] = "breakfast"

    bb["destination_str"] = "the dining table"

    bb["chosen_shelf"] = ""

    bb["chosen_shelf_str"] = ""

    sm = ServeBreakfast()

    outcome = sm(bb)

    yasmin.YASMIN_LOG_INFO(f"ServeBreakfast finished with outcome: {outcome}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
