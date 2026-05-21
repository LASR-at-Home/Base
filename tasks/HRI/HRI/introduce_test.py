#!/usr/bin/env python3

import rclpy

from geometry_msgs.msg import Point, PointStamped

from std_msgs.msg import Header

from HRI.states.introduce import Introduce


def main():
    rclpy.init()

    node = rclpy.create_node("introduce_test_node")

    sm = Introduce(node=node, guest_to_introduce="guest1")

    sm.userdata.guest_data = {
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
    sm.userdata.guest_seat_point = PointStamped(
        header=header,
        point=Point(x=6.654674, y=7.160965, z=1.5),
    )

    # Sitting person (host)
    sm.userdata.seated_guest_locs = [
        # Point(x=7.573320, y=7.281600, z=1.2),
        Point(x=1.5, y=-0.5, z=1.2),
    ]

    outcome = sm.execute()

    node.get_logger().info(f"Introduce finished with outcome: {outcome}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
