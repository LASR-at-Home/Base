#!/usr/bin/env python3
import rospy
from receptionist.state_machine import Receptionist
import smach
import smach_ros

from geometry_msgs.msg import Pose, Point, Quaternion, Polygon, PolygonStamped
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.validation import explain_validity

from std_msgs.msg import Header

if __name__ == "__main__":
    rospy.init_node("receptionist_robocup")

    wait_area_publisher = rospy.Publisher(
        "/receptionist/wait_area", PolygonStamped, queue_size=1, latch=True
    )

    seat_area_publisher = rospy.Publisher(
        "/receptionist/seat_area", PolygonStamped, queue_size=1, latch=True
    )

    sofa_area_publisher = rospy.Publisher(
        "/receptionist/sofa_area", PolygonStamped, queue_size=1, latch=True
    )

    wait_pose_param = rospy.get_param("/receptionist/wait_pose")

    wait_pose = Pose(
        position=Point(**wait_pose_param["position"]),
        orientation=Quaternion(**wait_pose_param["orientation"]),
    )

    wait_area_param = rospy.get_param("/receptionist/wait_area")
    wait_area = ShapelyPolygon(wait_area_param)

    table_pose_param = rospy.get_param("/receptionist/table_pose")
    table_pose = Pose(
        position=Point(**table_pose_param["position"]),
        orientation=Quaternion(**table_pose_param["orientation"]),
    )

    seat_pose_param = rospy.get_param("/receptionist/seat_pose")
    seat_pose = Pose(
        position=Point(**seat_pose_param["position"]),
        orientation=Quaternion(**seat_pose_param["orientation"]),
    )

    seat_area_param = rospy.get_param("/receptionist/seat_area")

    sofa_area_param = rospy.get_param("/receptionist/sofa_area")
    left_sofa_area_param = rospy.get_param("/receptionist/left_sofa_area")
    right_sofa_area_param = rospy.get_param("/receptionist/right_sofa_area")

    sofa_point_param = rospy.get_param("/receptionist/sofa_point")

    max_people_on_sofa = rospy.get_param("/receptionist/max_people_on_sofa")

    table_area_param = rospy.get_param("/receptionist/table_area")
    table_point_param = rospy.get_param("/receptionist/table_point")
    left_table_area_param = rospy.get_param("/receptionist/left_table_area")
    right_table_area_param = rospy.get_param("/receptionist/right_table_area")
    centre_table_area_param = rospy.get_param("/receptionist/centre_table_area")

    seat_area = ShapelyPolygon(seat_area_param)
    assert seat_area.is_valid, "Seat area is not valid"

    sofa_area = ShapelyPolygon(sofa_area_param)
    left_sofa_area = ShapelyPolygon(left_sofa_area_param)
    right_sofa_area = ShapelyPolygon(right_sofa_area_param)
    sofa_area_publisher.publish(
        PolygonStamped(
            polygon=Polygon(
                points=[Point(x=x, y=y, z=0.0) for (x, y) in sofa_area.exterior.coords]
            ),
            header=Header(frame_id="map"),
        )
    )
    assert sofa_area.is_valid, f"Sofa area is not valid: {explain_validity(sofa_area)}"

    sofa_point = Point(**sofa_point_param)
    table_area = ShapelyPolygon(table_area_param)
    left_table_area = ShapelyPolygon(left_table_area_param)
    right_table_area = ShapelyPolygon(right_table_area_param)
    centre_table_area = ShapelyPolygon(centre_table_area_param)
    table_point = Point(**table_point_param)
    # exclude the sofa area from the seat area
    # seat_area = seat_area.difference(sofa_area)
<<<<<<< HEAD

    search_motions = rospy.get_param("/receptionist/search_motions")
=======
>>>>>>> origin/main

    sweep = rospy.get_param("/receptionist/sweep")

    wait_area_publisher.publish(
        PolygonStamped(
            polygon=Polygon(
                points=[Point(x=x, y=y, z=0.0) for (x, y) in wait_area.exterior.coords]
            ),
            header=Header(frame_id="map"),
        )
    )

    seat_area_publisher.publish(
        PolygonStamped(
            polygon=Polygon(
                points=[Point(x=x, y=y, z=0.0) for (x, y) in seat_area.exterior.coords]
            ),
            header=Header(frame_id="map"),
        )
    )
    sofa_area_publisher.publish(
        PolygonStamped(
            polygon=Polygon(
                points=[Point(x=x, y=y, z=0.0) for (x, y) in sofa_area.exterior.coords]
            ),
            header=Header(frame_id="map"),
        )
    )

    assert seat_area.is_valid

    receptionist = Receptionist(
        wait_pose,
        wait_area,
        table_pose,
        table_point,
        table_area,
        left_table_area,
        right_table_area,
        centre_table_area,
        seat_pose,
        seat_area,
        sofa_area,
        left_sofa_area,
        right_sofa_area,
        sofa_point,
        {
            "name": "john",
            "drink": "milk",
            "interest": "robots",
            "dataset": "receptionist",
            "detection": False,
            "seating_detection": False,
        },
        max_people_on_sofa=max_people_on_sofa,
    )

    outcome = receptionist.execute()

    rospy.loginfo(f"Receptionist finished with outcome: {outcome}")
    rospy.spin()
