#!/usr/bin/env python3
import rclpy
import yasmin
import yasmin_ros
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from HRI.states.introduce import Introduce


def main():
    rclpy.init()
    yasmin_ros.set_ros_loggers()

    sm = Introduce(guest_to_introduce="guest1")

    bb = yasmin.Blackboard()

    bb["guest_data"] = {
        "host": {
            "name": "Sophie",
            "drink": "cola",
            "interest": "reading",
            "seating_detection": False,
        },
        "guest1": {
            "name": "John",
            "drink": "water",
            "interest": "cycling",
            "seating_detection": False,
        },
    }

    # Standing person (guest1)
    header = Header()
    header.frame_id = "map"
    bb["guest_seat_point"] = PointStamped(
        header=header,
        point=Point(x=6.654674, y=7.160965, z=1.5),
    )

    # Sitting person (host)
    bb["seated_guest_locs"] = [
        Point(x=1.5, y=-0.5, z=1.2),
    ]

    bb["person_index"] = 0

    outcome = sm(bb)
    yasmin.YASMIN_LOG_INFO(f"Introduce finished with outcome: {outcome}")

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()