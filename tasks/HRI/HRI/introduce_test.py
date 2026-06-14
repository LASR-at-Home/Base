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

    # Simulates what GREET would populate
    bb["guest_data"] = {
        "host": {
            "name": "Sophie",
            "drink": "cola",
            "detection": False,
            "seating_detection": False,
        },
        "guest1": {
            "name": "John",
            "drink": "water",
            "detection": False,
            "seating_detection": False,
        },
    }

    # Simulates what SEAT_GUEST would populate
    header = Header()
    header.frame_id = "map"

    # Where guest1 is sitting — robot looks here for second introduction
    bb["guest_seat_point"] = PointStamped(
        header=header,
        point=Point(x=1.98, y=0.546, z=0.00247),
    )

    # Where the host is sitting — robot looks here to recognise and introduce to
    bb["seated_guest_locs"] = [
        Point(x=2.0, y=-0.337, z=-0.00143),
    ]

    outcome = sm(bb)
    yasmin.YASMIN_LOG_INFO(f"Introduce finished with outcome: {outcome}")

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()