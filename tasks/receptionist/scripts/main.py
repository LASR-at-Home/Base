#!/usr/bin/env python3
import rospy
from receptionist.state_machine import Receptionist
import smach
import smach_ros

from geometry_msgs.msg import Pose, Point, Quaternion, Polygon, PolygonStamped
from shapely.geometry import Polygon as ShapelyPolygon

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

    seat_pose_param = rospy.get_param("/receptionist/seat_pose")
    seat_pose = Pose(
        position=Point(**seat_pose_param["position"]),
        orientation=Quaternion(**seat_pose_param["orientation"]),
    )

    seat_area_param = rospy.get_param("/receptionist/seat_area")

    sofa_area_param = rospy.get_param("/receptionist/sofa_area")

    sofa_point_param = rospy.get_param("/receptionist/sofa_point")

    max_people_on_sofa = rospy.get_param("/receptionist/max_people_on_sofa")

    seat_area = ShapelyPolygon(seat_area_param)
    assert seat_area.is_valid, "Seat area is not valid"

    sofa_area = ShapelyPolygon(sofa_area_param)
    sofa_area_publisher.publish(
        PolygonStamped(
            polygon=Polygon(
                points=[Point(x=x, y=y, z=0.0) for (x, y) in sofa_area.exterior.coords]
            ),
            header=Header(frame_id="map"),
        )
    )
    assert sofa_area.is_valid, "Sofa area is not valid"

    sofa_point = Point(**sofa_point_param)

    # exclude the sofa area from the seat area
    seat_area = seat_area.difference(sofa_area)

    search_motions = rospy.get_param("/receptionist/search_motions")

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
        seat_pose,
        search_motions,
        seat_area,
        sofa_area,
        sofa_point,
        {
            "name": "john",
            "drink": "milk",
            "dataset": "receptionist",
            "detection": False,
        },
        sweep=sweep,
        max_people_on_sofa=max_people_on_sofa,
    )

    outcome = receptionist.execute()

    rospy.loginfo(f"Receptionist finished with outcome: {outcome}")
    rospy.spin()
