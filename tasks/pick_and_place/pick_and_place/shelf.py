#!/usr/bin/env python3
import rclpy
import yasmin
import yasmin_ros
from pick_and_place.states.scan_shelves import ScanShelves
from pick_and_place.states.classify_category import ClassifyCategory
from pick_and_place.states.choose_shelf import ChooseShelf
from pick_and_place.states.instruct_place import InstructPlace

def main():
    rclpy.init()
    node = rclpy.create_node(
        "pick_and_place",  # ← match the config node name
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )
    yasmin_ros.set_ros_loggers(node)

    bb = yasmin.Blackboard()

    

    # ── Object to test placing ────────────────────────────────────────
    bb["object_name"]          = "coke"   # change to test different items
    bb["selected_object_name"] = "coke"
    bb["object_category"]      = ""
    bb["chosen_shelf"]         = ""
    bb["chosen_shelf_str"]     = ""
    bb["destination"]          = "cabinet"
    bb["destination_str"]      = "the cabinet"
    bb["shelf_data"]           = {}
    bb["shelf_category"]       = ""
    bb["object_names"]         = []
    bb["detected_objects"]     = []
    bb["debug_images"]         = []
    # ─────────────────────────────────────────────────────────────────

    # Step 1 — Scan shelves (robot must be in front of cabinet)
    print("Scanning shelves...")
    scanner = ScanShelves()
    outcome = scanner.execute(bb)
    print(f"ScanShelves outcome: {outcome}")
    print(f"shelf_data: {bb['shelf_data']}")

    if outcome == "failed":
        print("Scan failed — check YOLO service and robot position.")
        rclpy.shutdown()
        return

    # Step 2 — Classify
    classifier = ClassifyCategory(task="object")
    outcome = classifier.execute(bb)
    print(f"ClassifyCategory outcome: {outcome}")
    print(f"object_category: {bb['object_category']}")

    # Step 3 — Choose shelf
    chooser = ChooseShelf()
    outcome = chooser.execute(bb)
    print(f"ChooseShelf outcome: {outcome}")
    print(f"chosen_shelf: {bb['chosen_shelf']}")
    print(f"chosen_shelf_str: {bb['chosen_shelf_str']}")

    # Step 4 — Instruct place
    placer = InstructPlace()
    outcome = placer.execute(bb)
    print(f"InstructPlace outcome: {outcome}")

    rclpy.shutdown()

if __name__ == "__main__":
    main()