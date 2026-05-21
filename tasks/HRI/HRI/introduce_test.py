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

            "drink": "coffee",

            "seating_detection": False,

        },

        "guest1": {

            "name": "John",

            "drink": "water",

            "seating_detection": False,

        },

    }

    # Where guest1 is seated — replace with actual coordinates from Gazebo

    header = Header()

    header.frame_id = "map"

    sm.userdata.guest_seat_point = PointStamped(

        header=header,

        point=Point(x=0.406857, y=1.399995, z=1.5),

    )

    # Where already seated guests are — replace with actual coordinates from Gazebo

    # Each Point is a location the robot will look at and try to recognise

    sm.userdata.seated_guest_locs = [

        Point(x=0.9, y=0.2, z=1.5)

    ]

    outcome = sm.execute()

    node.get_logger().info(f"Introduce finished with outcome: {outcome}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
