#!/usr/bin/env python3
import rclpy
import yasmin
import yasmin_ros
from pick_and_place.states.serve_breakfast import ServeBreakfast


from pick_and_place.state_machine import PickAndPlaceNode
 
def main():

    rclpy.init()

    node = PickAndPlaceNode()  # has allow_undeclared_parameters=True

    yasmin_ros.set_ros_loggers(node)
 
    bb = yasmin.Blackboard()

    bb["detected_objects"]     = []

    bb["selected_object"]      = None

    bb["selected_object_name"] = ""

    bb["object_name"]          = ""

    bb["object_category"]      = "breakfast"

    bb["destination_str"]      = "the dining table"

    bb["chosen_shelf"]         = ""

    bb["chosen_shelf_str"]     = ""

    bb["last_rgb_image"]       = None

    bb["debug_images"]         = []
 
    sm = ServeBreakfast()

    outcome = sm(bb)

    yasmin.YASMIN_LOG_INFO(f"ServeBreakfast finished with outcome: {outcome}")
 
    if rclpy.ok():

        node.destroy_node()

        rclpy.shutdown()
 