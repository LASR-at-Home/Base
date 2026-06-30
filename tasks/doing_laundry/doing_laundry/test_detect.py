import rclpy
import yasmin
import yasmin_ros
from doing_laundry.states.detect_objects import DetectObjects


def main():
    rclpy.init()
    yasmin_ros.set_ros_loggers()

    bb = yasmin.Blackboard()
    bb["detected_objects"] = []
    bb["debug_images"] = []

    state = DetectObjects()
    outcome = state.execute(bb)

    print(f"Outcome: {outcome}")
    print(f"Detected {len(bb['detected_objects'])} objects:")
    for obj in bb["detected_objects"]:
        print(f"  - {obj.name} (confidence: {obj.confidence:.2f})")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
